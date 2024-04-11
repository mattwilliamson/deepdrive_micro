#include "main.h"

// TODO: PID Controller for revolutions per second

uint8_t on = 0;
int8_t direction = 1;
uint pulses = 0;
int speed = 65000;
const uint max_speed = 65000;

rcl_publisher_t publisher;
// std_msgs__msg__Int32 msgOut;
control_msgs__msg__MecanumDriveControllerState msgOut;

// Set up subscriber
rcl_subscription_t subscriber;
// std_msgs__msg__Int32 cmd;
control_msgs__msg__MecanumDriveControllerState cmd;

LEDRing ledRing = LEDRing(LED_RING_PIN, LED_RING_PIO, LED_RING_NUM_PIXELS);
StatusManager& status = StatusManager::getInstance();
std::vector<Motor*> motors(MOTOR_COUNT);


void core1_entry() {
  // Run the led ring animation loop on the other core in parallel
  // multicore_fifo_push_blocking(FLAG_VALUE);

  // Wait for first status push
  // uint32_t g = multicore_fifo_pop_blocking();
  // uint32_t g = (uint32_t)Status::Init;
  bool led_on = true;

  #ifndef LED_RING_ENABLED
  ledRing.off();
  return;
  #endif

  while (true) {
    led_on = !led_on;
    ledRing.renderStatus(status.get());
    led_status_set(led_on);
    sleep_ms(LED_RING_DELAY_MS);
    // sleep_ms(100);

    // Wait for next status push
    // Might not actually need IPC here
    // if (multicore_fifo_pop_timeout_us(0, &g)) {
    //   ledRing.resetAnimation();
    // }
  }
}

void blink_error() {
  led_status_blink(1000, 300, 100);
}

void timer_cb_general(rcl_timer_t *timer, int64_t last_call_time) {
    // msgOut.header.stamp.sec = 
    msgOut.front_left_wheel_velocity = motors[IDX_MOTOR_FRONT_LEFT]->getPulses();
    msgOut.front_right_wheel_velocity = motors[IDX_MOTOR_FRONT_RIGHT]->getPulses();
    msgOut.back_left_wheel_velocity = motors[IDX_MOTOR_BACK_LEFT]->getPulses();
    msgOut.back_right_wheel_velocity = motors[IDX_MOTOR_BACK_RIGHT]->getPulses();
    // msgOut.data.capacity = 4;
    // msgOut.data.size = 4;
    // msgOut.data.data[IDX_MOTOR_FRONT_LEFT] = motors[IDX_MOTOR_FRONT_LEFT]->getPulses();
    // msgOut.data.data[IDX_MOTOR_FRONT_RIGHT] = motors[IDX_MOTOR_FRONT_RIGHT]->getPulses();
    // msgOut.data.data[IDX_MOTOR_BACK_LEFT] = motors[IDX_MOTOR_BACK_LEFT]->getPulses();
    // msgOut.data.data[IDX_MOTOR_BACK_RIGHT] = motors[IDX_MOTOR_BACK_RIGHT]->getPulses();
    // msgOut.data = motors[IDX_MOTOR_FRONT_LEFT]->getPulses();

  RCSOFTCHECK(rcl_publish(&publisher, &msgOut, NULL));

  // if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
  //   // Lost connection to agent. Stop motors.
  //   for(auto &motor : motors) {
  //     motor->setSpeed(0);
  //   }
  //   status.set(Status::Error);
  //   printf("micro-ROS agent has stopped. Exiting...\n");
  //   sleep_ms(1000);
  //   exit(1);
  // } else {
  //   printf("Agent is still up!\n\n");
  // }
}

// void timer_cb_led_ring(rcl_timer_t *timer, int64_t last_call_time) {
//   static uint32_t t = 0;
//   led_ring_pattern_snakes(LED_RING_NUM_PIXELS, t);
//   t++;
// }

// Subscriber callback
void subscription_callback(const void *msgIn) {
  status.set(Status::Active);

  // Cast received message to used type

  const control_msgs__msg__MecanumDriveControllerState *m = (const control_msgs__msg__MecanumDriveControllerState *)msgIn;
  motors[IDX_MOTOR_FRONT_LEFT]->setSpeed(m->front_left_wheel_velocity);
  motors[IDX_MOTOR_FRONT_RIGHT]->setSpeed(m->front_right_wheel_velocity);
  motors[IDX_MOTOR_BACK_LEFT]->setSpeed(m->back_left_wheel_velocity);
  motors[IDX_MOTOR_BACK_RIGHT]->setSpeed(m->back_right_wheel_velocity);
  // for (size_t i = 0; i < motors.size(); i++) {
  //   if (i < m->data.size) {
  //     motors[i]->setSpeed(m->data.data[i]);
  //   }
  // }
  
  // Cast received message to used type
  // const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgIn;
  // motors[IDX_MOTOR_FRONT_LEFT]->setSpeed(m->data);
}

int main() {
  ledRing.start();
  status.set(Status::Connecting);
  led_status_init();

  // Setup 4 ESC brushless motor controllers
  motors[IDX_MOTOR_FRONT_LEFT] = new Motor(PIN_MOTOR_FRONT_LEFT, PIN_ENCODER_FRONT_LEFT);
  motors[IDX_MOTOR_BACK_LEFT] = new Motor(PIN_MOTOR_BACK_LEFT, PIN_ENCODER_BACK_LEFT);
  motors[IDX_MOTOR_FRONT_RIGHT] = new Motor(PIN_MOTOR_FRONT_RIGHT, PIN_ENCODER_FRONT_RIGHT);
  motors[IDX_MOTOR_BACK_RIGHT] = new Motor(PIN_MOTOR_BACK_RIGHT, PIN_ENCODER_BACK_RIGHT);

  // Parallel processing core - Currently just animates RGB LED ring
  multicore_launch_core1(core1_entry);

  status.set(Status::Connecting);

  #ifdef WATCHDOG_ENABLED
  // We rebooted because we got stuck or something
  if (watchdog_caused_reboot()) {
    // printf("Rebooted by Watchdog!\n");
    status.set(STATUS_REBOOTED);
    sleep_ms(10000);
  }

  // Enable the watchdog, requiring the watchdog to be updated every 100ms or the chip will reboot
  // second arg is pause on debug which means the watchdog will pause when stepping through code
  watchdog_enable(1000, false);
  #endif

  // Setup micro-ROS
  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);

  // ROS2 Node
  rcl_timer_t timer;
  // rcl_timer_t timer_led_ring;
  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rclc_executor_t executor;

  // Number of handles allowed in the executor (2 timer, 1 subscription)
  // Make sure to update as more are added!
  const size_t handles = 3;

  allocator = rcl_get_default_allocator();
  executor = rclc_executor_get_zero_initialized_executor();

  // Wait for agent successful ping for 2 minutes.
  // Connect to agent and init node
  // Check once and fail quickly if agent is not reachable to show error
  // RCSOFTCHECK(rmw_uros_ping_agent(30, 1));

  // Then retry until agent is reachable and fail hard
  RCCHECK(rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS));

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  status.set(Status::Connected);
  
  RCCHECK(rclc_node_init_default(&node, "deepdrive_micro_node", "", &support));

  // Publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, MecanumDriveControllerState),
      "deepdrive_micro/pulses"));

  // Timer
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100),
                                  timer_cb_general));

  // LED Ring Timer
  // RCCHECK(rclc_timer_init_default(&timer_led_ring, &support, RCL_MS_TO_NS(100),
                                  // timer_cb_led_ring));

  RCCHECK(rclc_executor_init(&executor, &support.context, handles, &allocator));
  
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer_led_ring));

  // Subscriber
  // ros2 topic pub --once deepdrive_micro/cmd std_msgs/msg/Int32 "{data:
  // 65000}"
  
  // RCCHECK(rclc_subscription_init_best_effort(
  //     &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //     "deepdrive_micro/cmd"));
  RCCHECK(rclc_subscription_init_best_effort(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, MecanumDriveControllerState),
      "deepdrive_micro/cmd"));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd,
                                         &subscription_callback, ON_NEW_DATA));
  
  RCCHECK(rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS));
  while (true) {
    #ifdef WATCHDOG_ENABLED
    watchdog_update();
    #endif
    
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }
  // rclc_executor_spin(&executor);

  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  RCCHECK(rcl_node_fini(&node));

  return 0;
}