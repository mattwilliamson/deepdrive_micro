#include "main.h"
#include "imu.h"
// #include "imu/driver_mpu9250_interface.h"
// #include "imu/driver_mpu9250_dmp.h"

// Number of uRosHandles allocated in the executor (1 timer, 1 subscription, 5 publisher)
const size_t uRosHandles = 7;

rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

rcl_publisher_t publisher_motor;
rcl_publisher_t publisher_battery;
rcl_publisher_t publisher_join_state;
rcl_publisher_t publisher_imu;

control_msgs__msg__MecanumDriveControllerState mgs_out_motor;
sensor_msgs__msg__BatteryState msg_out_battery;
sensor_msgs__msg__JointState *msg_out_joint_state;
sensor_msgs__msg__Imu *msg_out_imu;

rcl_subscription_t subscriber_motor;
control_msgs__msg__MecanumDriveControllerState msg_in_motor;
// maybe switch to control_msgs__msg__SteeringControllerStatus

IMU imu = IMU();

LEDRing led_ring = LEDRing(LED_RING_PIN, LED_RING_PIO, LED_RING_NUM_PIXELS);
StatusManager& status = StatusManager::getInstance();
std::vector<Motor*> motors(MOTOR_COUNT);

// TODO: Do I need to publish trajectory_msgs__msg__JointTrajectoryPoint ?

volatile bool render_led_ring = false;
volatile bool control_due = false;

repeating_timer_t timer_control;
repeating_timer_t timer_led_ring;

AnalogSensors* analog_sensors;

bool trigger_control(repeating_timer_t *rt) {
  control_due = true;
  return true;
}

bool trigger_led_ring(repeating_timer_t *rt) {
  render_led_ring = true;
  return true;
}

// Second core worker function
// LED Ring and control loop
void core1_entry() {
  bool led_on = true;

  #ifndef LED_RING_ENABLED
  led_ring.off();
  return;
  #endif

  while (true) {

    // Control loop for motors
    if (control_due) {
      control_due = false;

      // Read any motor encoder pulses
      for (auto& motor : motors) {
        // Calculate instantaneous speed and PID controller output
        int16_t pid_output = motor->calculatePid();

        if (motor->getTargetSpeed() == 0) {
          // We're supposed to be stopped, so let reset it all
          motor->stop();
          motor->pidController_->reset();
        } else {
          // Set the motor speed to the output the PID controller calculated
          motor->setSpeed(pid_output);
        }
      }
    }

    // LED Ring Loop
    if (render_led_ring) {
      render_led_ring = false;
      led_on = !led_on;
      led_ring.renderStatus(status.get());
      led_status_set(led_on);
    }

    // Sit tight until we have more work to do
    // tight_loop_contents();
    sleep_ms(1);
  }
}

// void publishDiagnosticMessage(const std::string& message) {
//   float temp = analog_sensors->getTemperature();
//   float battery = analog_sensors->getBatteryVoltage();
  
//   diagnostic_msgs__msg__DiagnosticStatus diagnostic_msg;
//   diagnostic_msg.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
//   diagnostic_msg.message = message;

//   // Publish the diagnostic message
//   rcl_publisher_t diagnostic_publisher;
//   RCCHECK(rclc_publisher_init_default(
//     &diagnostic_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus),
//     "diagnostics"));
//   RCSOFTCHECK(rcl_publish(&diagnostic_publisher, &diagnostic_msg, NULL));
//   RCCHECK(rcl_publisher_fini(&diagnostic_publisher, &node));
// }

int init_battery() {
  RCCHECK(rclc_publisher_init_default(
      &publisher_battery, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      "deepdrive_micro/battery"));

  micro_ros_string_utilities_set(msg_out_battery.location, "base_link");
  micro_ros_string_utilities_set(msg_out_battery.serial_number,  "1234567890");

  return 0;
}

void publish_battery() {
  msg_out_battery.voltage = analog_sensors->getBatteryVoltage();
  msg_out_battery.percentage = AnalogSensors::convertVoltageToPercentage(msg_out_battery.voltage);
  msg_out_battery.design_capacity = 5200;
  msg_out_battery.present = true;
  msg_out_battery.temperature = analog_sensors->getTemperature();
  
  msg_out_battery.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;

  RCSOFTCHECK(rcl_publish(&publisher_battery, &msg_out_battery, NULL));
}

int init_joint_state()  {
  RCCHECK(rclc_publisher_init_default(
      &publisher_join_state, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "deepdrive_micro/joint_state"));

   msg_out_joint_state = sensor_msgs__msg__JointState__create();
   micro_ros_string_utilities_set(msg_out_joint_state->header.frame_id, "base_link");
  
  msg_out_joint_state->name = *rosidl_runtime_c__String__Sequence__create(MOTOR_COUNT);
  rosidl_runtime_c__float64__Sequence__init(&msg_out_joint_state->position, MOTOR_COUNT);
  rosidl_runtime_c__float64__Sequence__init(&msg_out_joint_state->velocity, MOTOR_COUNT);
  rosidl_runtime_c__float64__Sequence__init(&msg_out_joint_state->effort, MOTOR_COUNT);

  msg_out_joint_state->name.data[IDX_MOTOR_FRONT_LEFT] = micro_ros_string_utilities_init(MOTOR_JOIN_FRONT_LEFT);
  msg_out_joint_state->name.data[IDX_MOTOR_BACK_LEFT] = micro_ros_string_utilities_init(MOTOR_JOIN_BACK_LEFT);
  msg_out_joint_state->name.data[IDX_MOTOR_FRONT_RIGHT] = micro_ros_string_utilities_init(MOTOR_JOIN_FRONT_RIGHT);
  msg_out_joint_state->name.data[IDX_MOTOR_BACK_RIGHT] = micro_ros_string_utilities_init(MOTOR_JOIN_BACK_RIGHT);

  return 0;
}

void publish_joint_state() {
  /**
  * The state of each joint (revolute or prismatic) is defined by:
  *  * the position of the joint (rad or m),
  *  * the velocity of the joint (rad/s or m/s) and
  *  * the effort that is applied in the joint (Nm or N).
  */
  msg_out_joint_state->header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  msg_out_joint_state->header.stamp.nanosec = rmw_uros_epoch_nanos();

  // TODO: Actually populate
  for (int i = 0; i < MOTOR_COUNT; i++) {
    msg_out_joint_state->position.data[i] = motors[i]->getSpeedMetersPerSecond();
    msg_out_joint_state->velocity.data[i] = motors[i]->getSpeedMetersPerSecond();
    msg_out_joint_state->effort.data[i] = motors[i]->getSpeedMetersPerSecond();
  }

  RCSOFTCHECK(rcl_publish(&publisher_join_state, msg_out_joint_state, NULL));
}


int init_imu()  {
  RCCHECK(rclc_publisher_init_default(
      &publisher_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "deepdrive_micro/imu"));

  if(imu.start() != 0) {
    printf("IMU start failed\r\n");
    return 1;
  }

   msg_out_imu = sensor_msgs__msg__Imu__create();
   msg_out_imu->header.frame_id = micro_ros_string_utilities_init("base_link");
  //  rosidl_runtime_c__float64__Sequence__init(&msg_out_imu->effort, MOTOR_COUNT);


  return 0;
}

void publish_imu() {
  sleep_ms(10);
  msg_out_imu->header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  msg_out_imu->header.stamp.nanosec = rmw_uros_epoch_nanos();

  icm20984_data_t *d = imu.read();
  sleep_ms(10);
  // printf("accel. x: %d, y: %d, z:%d\r\n", d->accel_raw[0], d->accel_raw[1], d->accel_raw[2]);
  // printf("gyro.  x: %d, y: %d, z:%d\r\n", d->gyro_raw[0], d->gyro_raw[1], d->gyro_raw[2]);
  // printf("mag.   x: %d, y: %d, z:%d\r\n", d->mag_raw[0], d->mag_raw[1], d->mag_raw[2]);

  // TODO: Covariance
  // TODO: Switch these to real values
  // msg_out_imu->linear_acceleration_covariance
  msg_out_imu->linear_acceleration.x = d->accel_raw[0];
  msg_out_imu->linear_acceleration.y = d->accel_raw[1];
  msg_out_imu->linear_acceleration.z = d->accel_raw[2];

  // msg_out_imu->angular_velocity_covariance = 0.001;
  msg_out_imu->angular_velocity.x = d->gyro_raw[0];
  msg_out_imu->angular_velocity.y = d->gyro_raw[1];
  msg_out_imu->angular_velocity.z = d->gyro_raw[2];

  // TODO: Publish mag
  // msg_out_imu->orientation_covariance = 0.001;
  msg_out_imu->orientation.x = d->mag_raw[0];
  msg_out_imu->orientation.y = d->mag_raw[1];
  msg_out_imu->orientation.z = d->mag_raw[2];
  
  RCSOFTCHECK(rcl_publish(&publisher_imu, msg_out_imu, NULL));
}



void timer_cb_general(rcl_timer_t *timer, int64_t last_call_time) {
    mgs_out_motor.front_left_wheel_velocity = motors[IDX_MOTOR_FRONT_LEFT]->getSpeedMetersPerSecond();
    mgs_out_motor.front_right_wheel_velocity = motors[IDX_MOTOR_FRONT_RIGHT]->getSpeedMetersPerSecond();
    mgs_out_motor.back_left_wheel_velocity = motors[IDX_MOTOR_BACK_LEFT]->getSpeedMetersPerSecond();
    mgs_out_motor.back_right_wheel_velocity = motors[IDX_MOTOR_BACK_RIGHT]->getSpeedMetersPerSecond();

    // mgs_out_motor.front_left_wheel_velocity = motors[IDX_MOTOR_FRONT_LEFT]->getSpeedcmd();
    // mgs_out_motor.front_right_wheel_velocity = motors[IDX_MOTOR_FRONT_RIGHT]->getSpeedcmd();
    // mgs_out_motor.back_left_wheel_velocity = motors[IDX_MOTOR_BACK_LEFT]->getSpeedcmd();
    // mgs_out_motor.back_right_wheel_velocity = motors[IDX_MOTOR_BACK_RIGHT]->getSpeedcmd();

  // float temp = analog_sensors->getTemperature();
  // float battery = analog_sensors->getBatteryVoltage();
  // mgs_out_motor.reference_velocity.linear.x = temp;
  // mgs_out_motor.reference_velocity.linear.y = battery;
  // x -1479.795166015625
  // y 3.2991943359375

  RCSOFTCHECK(rcl_publish(&publisher_motor, &mgs_out_motor, NULL));

  // if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
  //   // Lost connection to agent. Stop motors.
  //   for(auto &motor : motors) {
  //     motor->setSpeed(0);
  //   }
  //   status.set(Status::Error);
  //   printf("micro-ROS agent has stopped. Exiting...\r\n");
  //   sleep_ms(1000);
  //   exit(1);
  // } else {
  //   printf("Agent is still up!\r\n\r\n");
  // }

  // TODO: Move some of these to config
  publish_battery();

  publish_joint_state();

  publish_imu();
}

// void timer_cb_led_ring(rcl_timer_t *timer, int64_t last_call_time) {
//   static uint32_t t = 0;
//   led_ring_pattern_snakes(LED_RING_NUM_PIXELS, t);
//   t++;
// }

// Subscriber callback
void subscription_motor_callback(const void *msgIn) {
  status.set(Status::Active);

  // Cast received message to used type

  const control_msgs__msg__MecanumDriveControllerState *m = (const control_msgs__msg__MecanumDriveControllerState *)msgIn;
  motors[IDX_MOTOR_FRONT_LEFT]->setTargetSpeed(m->front_left_wheel_velocity);
  motors[IDX_MOTOR_FRONT_RIGHT]->setTargetSpeed(m->front_right_wheel_velocity);
  motors[IDX_MOTOR_BACK_LEFT]->setTargetSpeed(m->back_left_wheel_velocity);
  motors[IDX_MOTOR_BACK_RIGHT]->setTargetSpeed(m->back_right_wheel_velocity);
}

void setup_watchdog() {
#ifdef WATCHDOG_ENABLED
  // We rebooted because we got stuck or something
  if (watchdog_caused_reboot()) {
    // printf("Rebooted by Watchdog!\r\n");
    status.set(STATUS_REBOOTED);
    sleep_ms(10000);
  }

  // Enable the watchdog, requiring the watchdog to be updated every 100ms or the chip will reboot
  // second arg is pause on debug which means the watchdog will pause when stepping through code
  watchdog_enable(1000, false);
#endif
}



#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "imu.h"


int main() {

  // stdio_init_all(); // Called by uros
  setup_watchdog();
  led_ring.start();
  led_status_init();

  analog_sensors->init();

  status.set(Status::Connecting);

  // Setup 4 ESC brushless motor controllers
  motors[IDX_MOTOR_FRONT_LEFT] = new Motor(PIN_MOTOR_FRONT_LEFT, PIN_ENCODER_FRONT_LEFT);
  motors[IDX_MOTOR_BACK_LEFT] = new Motor(PIN_MOTOR_BACK_LEFT, PIN_ENCODER_BACK_LEFT);
  motors[IDX_MOTOR_FRONT_RIGHT] = new Motor(PIN_MOTOR_FRONT_RIGHT, PIN_ENCODER_FRONT_RIGHT);
  motors[IDX_MOTOR_BACK_RIGHT] = new Motor(PIN_MOTOR_BACK_RIGHT, PIN_ENCODER_BACK_RIGHT);

  // Parallel processing core - RGB LED Ring and control loop
  multicore_launch_core1(core1_entry);
  
  // Setup control loop timer
  if (!add_repeating_timer_us(-MICROSECONDS / CONTROL_LOOP_HZ, trigger_control, NULL, &timer_control)) {
    // printf("Failed to add control loop timer\r\n");
    return 1;
  }

  // Setup LED Ring animation loop timer
  if (!add_repeating_timer_us(-MICROSECONDS / LED_RING_HZ, trigger_led_ring, NULL, &timer_led_ring)) {
    // printf("Failed to add led ring timer\r\n");
    return 1;
  }

  // -------------------------
  // Setup micro-ROS
  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);

  allocator = rcl_get_default_allocator();
  executor = rclc_executor_get_zero_initialized_executor();

  // Wait for agent successful ping for 2 minutes.
  RCCHECK(rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS));

  status.set(Status::Connected);

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  RCCHECK(rclc_node_init_default(&node, "deepdrive_micro_node", "", &support));

  init_imu();

  // TODO: Define topics in config
  // Publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher_motor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, MecanumDriveControllerState),
      "deepdrive_micro/pulses"));

  if (init_battery() != 0) {return 1;};
  if (init_joint_state() != 0) {return 1;};

  // Timer
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1.0/CONTROL_LOOP_HZ*1000),
                                  timer_cb_general));

  RCCHECK(rclc_executor_init(&executor, &support.context, uRosHandles, &allocator));
  
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCCHECK(rclc_subscription_init_best_effort(
      &subscriber_motor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, MecanumDriveControllerState),
      "deepdrive_micro/cmd"));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_motor, &msg_in_motor, &subscription_motor_callback, ON_NEW_DATA));
  
  while (true) {
    #ifdef WATCHDOG_ENABLED
    watchdog_update();
    #endif
    
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000)));
    // RCCHECK(rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS));
  }
  // rclc_executor_spin(&executor);

  RCCHECK(rcl_publisher_fini(&publisher_motor, &node));
  RCCHECK(rcl_subscription_fini(&subscriber_motor, &node));
  RCCHECK(rcl_node_fini(&node));

  return 0;
}