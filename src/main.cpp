#include "main.h"

#ifdef IMU_ENABLED
#include "imu.h"
#endif

// #include "imu/driver_mpu9250_interface.h"
// #include "imu/driver_mpu9250_dmp.h"

// Number of uRosHandles allocated in the executor (1 timer, 1 subscription, 6 publisher)
const size_t uRosHandles = 12 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES;

uint64_t core_start[2] = {0, 0};
uint64_t core_elapsed[2] = {0, 0};

rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rclc_parameter_server_t param_server;

rcl_publisher_t publisher_motor;
rcl_publisher_t publisher_battery;
rcl_publisher_t publisher_join_state;
rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_mag;
rcl_publisher_t publisher_diagnostic;
rcl_publisher_t publisher_odom;

control_msgs__msg__MecanumDriveControllerState mgs_out_motor;
sensor_msgs__msg__BatteryState msg_out_battery;
sensor_msgs__msg__JointState *msg_out_joint_state;
sensor_msgs__msg__Imu *msg_out_imu;
sensor_msgs__msg__MagneticField *msg_out_mag;
diagnostic_msgs__msg__DiagnosticArray msg_out_diagnostic;
nav_msgs__msg__Odometry *msg_out_odom;

rcl_subscription_t subscriber_motor;
control_msgs__msg__MecanumDriveControllerState msg_in_motor;

rcl_subscription_t subscriber_cmd_vel;
geometry_msgs__msg__Twist msg_in_cmd_vel;


#ifdef IMU_ENABLED
IMU imu = IMU();
#endif

LEDRing led_ring = LEDRing(LED_RING_PIN, LED_RING_PIO, LED_RING_NUM_PIXELS);
// status = StatusManager::getInstance();
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

  while (true) {
    core_start[1] = time_us_64();

    // Control loop for motors
    if (control_due) {
      control_due = false;

      // Read any motor encoder pulses
      for (auto& motor : motors) {
        motor->updateMotorOutput();
      }
      core_elapsed[1] = time_us_64() - core_start[1];
    }

    #ifndef LED_RING_ENABLED
    led_ring.off();
    #else
    // LED Ring Loop
    if (render_led_ring) {
      render_led_ring = false;
      led_on = !led_on;
      led_ring.renderStatus(status.get());
      led_status_set(led_on);
      core_elapsed[1] = time_us_64() - core_start[1];
    }
    #endif


    // Sit tight until we have more work to do
    tight_loop_contents();
    // sleep_ms(1);
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

// rclc_publisher_init_best_effort not currently working, but should enable async publishing

// Subscriber callback for cmd vel
void subscription_cmd_vel_callback(const void *msgIn) {
  status.set(Status::Active);

  // Cast received message to used type

  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgIn;
  for (auto &motor : motors) {
    motor->setTargetSpeedMeters(m->linear.x);
  }
}

int init_cmd_vel() {
  RCCHECK(rclc_subscription_init_best_effort(
      &subscriber_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "deepdrive_micro/cmd_vel"));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_cmd_vel, &msg_in_cmd_vel,
    &subscription_cmd_vel_callback, ON_NEW_DATA));

  return 0;
}

int init_diagnostic() {
  RCCHECK(rclc_publisher_init_default(
      &publisher_diagnostic, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
      "/diagnostics"));

  msg_out_diagnostic.header.frame_id = micro_ros_string_utilities_init(DIAGNOSTIC_FRAME);
  diagnostic_msgs__msg__DiagnosticStatus__Sequence__init(&msg_out_diagnostic.status, DIAGNOSTIC_COUNT);

  msg_out_diagnostic.status.data[0].hardware_id = micro_ros_string_utilities_init("deepdrive_micro");
  msg_out_diagnostic.status.data[0].name = micro_ros_string_utilities_init("CPU Loop Time (us)");
  
  diagnostic_msgs__msg__KeyValue__Sequence__init(&msg_out_diagnostic.status.data[0].values, 2);
  msg_out_diagnostic.status.data[0].values.data[0].key = micro_ros_string_utilities_init("Core 0");
  msg_out_diagnostic.status.data[0].values.data[1].key = micro_ros_string_utilities_init("Core 1");

  return 0;
}

void publish_diagnostic() {
  msg_out_diagnostic.header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  msg_out_diagnostic.header.stamp.nanosec = rmw_uros_epoch_nanos();

  // Convert core_elapsed[0] to string
  std::string core_elapsed_0_str = std::to_string(core_elapsed[0]);
  // Assign the converted value to value.data
  msg_out_diagnostic.status.data[0].values.data[0].value.data = const_cast<char*>(core_elapsed_0_str.c_str());

  // Convert core_elapsed[1] to string
  std::string core_elapsed_1_str = std::to_string(core_elapsed[1]);
  // Assign the converted value to value.data
  msg_out_diagnostic.status.data[0].values.data[1].value.data = const_cast<char*>(core_elapsed_1_str.c_str());

  // TODO: set error string from status
  msg_out_diagnostic.status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__OK;
  msg_out_diagnostic.status.data[0].message = micro_ros_string_utilities_init("OK");

  // TODO: Set status string
  switch(status.get()) {
    case Status::Connecting:
      msg_out_diagnostic.status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
      break;
    case Status::Connected:
      msg_out_diagnostic.status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__OK;
      break;
    case Status::Active:
      msg_out_diagnostic.status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__OK;
      break;
    case Status::Error:
      msg_out_diagnostic.status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
      msg_out_diagnostic.status.data[0].message = micro_ros_string_utilities_init("Error");
      break;
    case Status::Rebooted:
      msg_out_diagnostic.status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
      break;
  }
  
  // TODO: Send Status
  if (!rmw_uros_epoch_synchronized()) {
    msg_out_diagnostic.status.data[0].message = micro_ros_string_utilities_init("Time not synchronized");
  }

  RCSOFTCHECK(rcl_publish(&publisher_diagnostic, &msg_out_diagnostic, NULL));
}

int init_odom() {
  RCCHECK(rclc_publisher_init_default(
      &publisher_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "deepdrive_micro/odom"));

  // TODO: Make sure we got the frame and everything correct
  msg_out_odom = nav_msgs__msg__Odometry__create();
  msg_out_odom->header.frame_id = micro_ros_string_utilities_init("odom");
  msg_out_odom->child_frame_id = micro_ros_string_utilities_init("base_link");

  msg_out_odom->pose.covariance[0] = 0.001;
  msg_out_odom->pose.covariance[4] = 0.001;
  msg_out_odom->pose.covariance[8] = 0.001;

  return 0;
}

void publish_odom() {
  msg_out_odom->header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  msg_out_odom->header.stamp.nanosec = rmw_uros_epoch_nanos();

  // Get the meters traveled for each motor and average it out
  Meters meters = 0;
  for (auto &motor : motors) {
    meters += motor->getTotalMeters();
  }
  meters /= (double)MOTOR_COUNT;

  msg_out_odom->pose.pose.position.x = meters;

  msg_out_odom->pose.pose.orientation.x = motors[IDX_MOTOR_FRONT_LEFT]->getTotalMicrometers();
  msg_out_odom->pose.pose.orientation.y = motors[IDX_MOTOR_BACK_LEFT]->getTotalMicrometers();
  msg_out_odom->pose.pose.orientation.z = motors[IDX_MOTOR_FRONT_RIGHT]->getTotalMicrometers();
  msg_out_odom->pose.pose.orientation.w = motors[IDX_MOTOR_BACK_RIGHT]->getTotalMicrometers();

  RCSOFTCHECK(rcl_publish(&publisher_odom, msg_out_odom, NULL));
}


int init_battery() {
  RCCHECK(rclc_publisher_init_default(
      &publisher_battery, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      "deepdrive_micro/battery"));

  msg_out_battery.location = micro_ros_string_utilities_init("base_link");
  msg_out_battery.serial_number = micro_ros_string_utilities_init("1234567890");
  msg_out_battery.design_capacity = BATTERY_CAPACITY;
  msg_out_battery.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;

  return 0;
}

void publish_battery() {
  // TODO: Do this in the other core
  msg_out_battery.voltage = analog_sensors->getBatteryVoltage();
  msg_out_battery.percentage = AnalogSensors::convertVoltageToPercentage(msg_out_battery.voltage);
  
  msg_out_battery.present = true;
  msg_out_battery.temperature = analog_sensors->getTemperature();

  if (msg_out_battery.voltage > 2.0) {
    msg_out_battery.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    msg_out_battery.present = true;
  } else {
    msg_out_battery.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
    msg_out_battery.present = false;
  }

  RCSOFTCHECK(rcl_publish(&publisher_battery, &msg_out_battery, NULL));
}

int init_joint_state()  {
  // static rmw_subscription_allocation_t * allocation;

  RCCHECK(rclc_publisher_init_default(
      &publisher_join_state, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "deepdrive_micro/joint_state"));

// #define ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgSubfolder, MsgName) \
//   ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME( \
//     rosidl_typesupport_c, PkgName, MsgSubfolder, MsgName)()

//   RCCHECK(rmw_init_subscription_allocation(
//     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
//     ROSIDL_GET_SEQUENCE_BOUNDS(sensor_msgs, msg, JointState),
//     &allocation
//     ));

  msg_out_joint_state = sensor_msgs__msg__JointState__create();
  msg_out_joint_state->header.frame_id = micro_ros_string_utilities_init("base_link");
  
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

  for (int i = 0; i < MOTOR_COUNT; i++) {
    msg_out_joint_state->position.data[i] = motors[i]->getPosition();
    msg_out_joint_state->velocity.data[i] = motors[i]->getSpeedRadians();
    // msg_out_joint_state->effort.data[i] = motors[i]->getSpeedMetersPerSecond();
  }
  
  RCSOFTCHECK(rcl_publish(&publisher_join_state, msg_out_joint_state, NULL));
}


int init_imu()  {
  #ifdef IMU_ENABLED
  RCCHECK(rclc_publisher_init_default(
      &publisher_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "deepdrive_micro/imu"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_mag, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "deepdrive_micro/mag"));

  if(imu.start() != 0) {
    printf("IMU start failed\r\n");
    return 1;
  }

  // Calibrate the IMU
  imu.calibrate();
  // TODO: Save the compass calibration to EEPROM or flash or something and expose a service to trigger it

   msg_out_imu = sensor_msgs__msg__Imu__create();
   msg_out_imu->header.frame_id = micro_ros_string_utilities_init(IMU_FRAME);

  msg_out_mag = sensor_msgs__msg__MagneticField__create();
  msg_out_mag->header.frame_id = micro_ros_string_utilities_init(IMU_FRAME);

  #endif

  return 0;
}

void publish_imu() {
  #ifdef IMU_ENABLED
  int8_t success = imu.read();

  Vector3 accel = imu.getAccel();

  msg_out_imu->linear_acceleration.x = accel.x;
  msg_out_imu->linear_acceleration.y = accel.y;
  msg_out_imu->linear_acceleration.z = accel.z;

  msg_out_imu->linear_acceleration_covariance[0] = 0.001;
  msg_out_imu->linear_acceleration_covariance[4] = 0.001;
  msg_out_imu->linear_acceleration_covariance[8] = 0.001;

  Vector3 gyro = imu.getGyro();

  msg_out_imu->angular_velocity.x = gyro.x;
  msg_out_imu->angular_velocity.y = gyro.y;
  msg_out_imu->angular_velocity.z = gyro.z;

  msg_out_imu->angular_velocity_covariance[0] = 0.001;
  msg_out_imu->angular_velocity_covariance[4] = 0.001;
  msg_out_imu->angular_velocity_covariance[8] = 0.001;

  Quaternion orientation = imu.getOrientation();

  msg_out_imu->orientation.x = orientation.x;
  msg_out_imu->orientation.y = orientation.y;
  msg_out_imu->orientation.z = orientation.z;
  msg_out_imu->orientation.w = orientation.w;

  msg_out_imu->orientation_covariance[0] = 0.001;
  msg_out_imu->orientation_covariance[4] = 0.001;
  msg_out_imu->orientation_covariance[8] = 0.001;

  msg_out_imu->header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  msg_out_imu->header.stamp.nanosec = rmw_uros_epoch_nanos();
  
  RCSOFTCHECK(rcl_publish(&publisher_imu, msg_out_imu, NULL));

  // Magnetometer
  Vector3 mag = imu.getMag();

  msg_out_mag->magnetic_field.x = mag.x;
  msg_out_mag->magnetic_field.y = mag.y;
  msg_out_mag->magnetic_field.z = mag.z;

  msg_out_mag->magnetic_field_covariance[0] = 0.001;
  msg_out_mag->magnetic_field_covariance[4] = 0.001;
  msg_out_mag->magnetic_field_covariance[8] = 0.001;

  msg_out_mag->header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  msg_out_mag->header.stamp.nanosec = rmw_uros_epoch_nanos();

  RCSOFTCHECK(rcl_publish(&publisher_mag, msg_out_mag, NULL));
  #endif
}


void timer_cb_general(rcl_timer_t *timer, int64_t last_call_time) {
  // TODO: Do some calculations in the other core and publish in this one
  core_start[0] = time_us_64();

  mgs_out_motor.header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  mgs_out_motor.header.stamp.nanosec = rmw_uros_epoch_nanos();

  // 121us to calculate these
  mgs_out_motor.front_left_wheel_velocity = motors[IDX_MOTOR_FRONT_LEFT]->getSpeedMeters();
  mgs_out_motor.front_right_wheel_velocity = motors[IDX_MOTOR_FRONT_RIGHT]->getSpeedMeters();
  mgs_out_motor.back_left_wheel_velocity = motors[IDX_MOTOR_BACK_LEFT]->getSpeedMeters();
  mgs_out_motor.back_right_wheel_velocity = motors[IDX_MOTOR_BACK_RIGHT]->getSpeedMeters();

  // mgs_out_motor.front_left_wheel_velocity = motors[IDX_MOTOR_FRONT_LEFT]->getSpeedSignal();
  // mgs_out_motor.front_right_wheel_velocity = motors[IDX_MOTOR_FRONT_RIGHT]->getSpeedSignal();
  // mgs_out_motor.back_left_wheel_velocity = motors[IDX_MOTOR_BACK_LEFT]->getSpeedSignal();
  // mgs_out_motor.back_right_wheel_velocity = motors[IDX_MOTOR_BACK_RIGHT]->getSpeedSignal();

  // float temp = analog_sensors->getTemperature();
  // float battery = analog_sensors->getBatteryVoltage();
  // mgs_out_motor.reference_velocity.linear.x = temp;
  // mgs_out_motor.reference_velocity.linear.y = battery;
  // x -1479.795166015625
  // y 3.2991943359375

  // 6700us for publisher_motor
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
  
  // 13,470us to publish_joint_state
  publish_joint_state();


  // 630us to publish_imu
  publish_imu();

  publish_odom();

  core_elapsed[0] = time_us_64() - core_start[0];

  publish_diagnostic();
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
  motors[IDX_MOTOR_FRONT_LEFT]->setTargetSpeedMeters(m->front_left_wheel_velocity);
  motors[IDX_MOTOR_FRONT_RIGHT]->setTargetSpeedMeters(m->front_right_wheel_velocity);
  motors[IDX_MOTOR_BACK_LEFT]->setTargetSpeedMeters(m->back_left_wheel_velocity);
  motors[IDX_MOTOR_BACK_RIGHT]->setTargetSpeedMeters(m->back_right_wheel_velocity);
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

// Check if strings are the same
bool isDoubleNamed(const Parameter * new_param, const char *name) {
  if (new_param == NULL) {
    return false;
  }
  return strcmp(new_param->name.data, name) == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE;
}

bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context) {
  (void) context;
  if (new_param == NULL) {
    return false;
  }
  
  if (isDoubleNamed(new_param, PARAM_PID_KP)) {
    for (auto &motor : motors) {
      motor->pidController_->setKp(new_param->value.double_value);
    }
  } else if (isDoubleNamed(new_param, PARAM_PID_KI)) {
    for (auto &motor : motors) {
      motor->pidController_->setKi(new_param->value.double_value);
    }
  } else if (isDoubleNamed(new_param, PARAM_PID_KD)) {
    for (auto &motor : motors) {
      motor->pidController_->setKd(new_param->value.double_value);
    }
  } else {
    // Unknown param
    return false;
  }
  
  // int64_t old;
  // RCSOFTCHECK(rcl_timer_exchange_period(&timer, RCL_MS_TO_NS(new_param->value.integer_value), &old));
  // printf("Publish rate %ld ms\n", new_param->value.integer_value);
  return true;
}



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

  // Wait for agent successful ping for 2 minutes.
  RCCHECK(rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS));

  status.set(Status::Connected);

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  RCCHECK(rclc_node_init_default(&node, "deepdrive_micro_node", "", &support));

  // Synchronize time with the agent
  rmw_uros_sync_session(5000);

  // TODO: Define topics in config
  // Publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher_motor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, MecanumDriveControllerState),
      "deepdrive_micro/pulses"));

  if (init_battery() != 0) {return 1;};
  if (init_joint_state() != 0) {return 1;};
  init_imu();
  init_diagnostic();
  init_odom();

  // Timer
  // RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1.0/CONTROL_LOOP_HZ*1000),
  //                                 timer_cb_general));
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000),
                                  timer_cb_general));

  // TODO: Figure out why param server isn't working
  // https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk/issues/925
  // https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk/blob/e315a376cacb80fe2bddaa2b3028425e0cfa4dd1/libmicroros/include/rmw_microxrcedds_c/config.h#L55
  // https://micro.ros.org/docs/tutorials/programming_rcl_rclc/parameters/
  // https://stackoverflow.com/questions/67563340/how-to-pass-command-line-arguments-to-cmake-in-vscode
  // Create parameter service
  // const rclc_parameter_options_t param_server_options = {
  //     .notify_changed_over_dds = true,
  //     .max_params = 3,
  //     .allow_undeclared_parameters = true,
  //     .low_mem_mode = true
  //   };

  // rclc_parameter_server_init_with_option(&param_server, &node, &param_server_options);
  // rclc_parameter_server_init_default(&param_server, &node);

  // TODO: Make handles variable
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, uRosHandles, &allocator));

  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCCHECK(rclc_subscription_init_best_effort(
      &subscriber_motor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, MecanumDriveControllerState),
      "deepdrive_micro/cmd"));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_motor, &msg_in_motor,
    &subscription_motor_callback, ON_NEW_DATA));
  RCCHECK(init_cmd_vel());
  // TODO: Fix this. Probably memory issue.
  // RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed));

  // // Add parameters
  // rclc_add_parameter(&param_server, PARAM_PID_KP, RCLC_PARAMETER_DOUBLE);
  // rclc_add_parameter(&param_server, PARAM_PID_KI, RCLC_PARAMETER_DOUBLE);
  // rclc_add_parameter(&param_server, PARAM_PID_KD, RCLC_PARAMETER_DOUBLE);

  // // Add parameters constraints
  // rclc_add_parameter_description(&param_server, PARAM_PID_KP, "Motor PID Controller Kp value", "");
  // rclc_add_parameter_description(&param_server, PARAM_PID_KI, "Motor PID Controller Ki value", "");
  // rclc_add_parameter_description(&param_server, PARAM_PID_KD, "Motor PID Controller Kd value", "");

  // // Set parameter initial values
  // rclc_parameter_set_double(&param_server, PARAM_PID_KP, 0.1);
  // rclc_parameter_set_double(&param_server, PARAM_PID_KI, 0.0);
  // rclc_parameter_set_double(&param_server, PARAM_PID_KD, 0.0);
  
  // while (true) {
  //   #ifdef WATCHDOG_ENABLED
  //   watchdog_update();
  //   #endif
    
  //   RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  //   // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  //   // RCCHECK(rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS));
  // }
  
  RCCHECK(rclc_executor_spin(&executor));

  RCCHECK(rcl_publisher_fini(&publisher_motor, &node));
  RCCHECK(rcl_subscription_fini(&subscriber_motor, &node));
  RCCHECK(rcl_node_fini(&node));

  return 0;
}