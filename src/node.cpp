#include "node.hpp"

int Node::init_motors() {
  // Setup 4 ESC brushless motor controllers
  motors.resize(MOTOR_COUNT);
  motors[IDX_MOTOR_FRONT_LEFT] =
      new Motor(PIN_MOTOR_FRONT_LEFT, PIN_ENCODER_FRONT_LEFT);
  motors[IDX_MOTOR_BACK_LEFT] =
      new Motor(PIN_MOTOR_BACK_LEFT, PIN_ENCODER_BACK_LEFT);
  motors[IDX_MOTOR_FRONT_RIGHT] =
      new Motor(PIN_MOTOR_FRONT_RIGHT, PIN_ENCODER_FRONT_RIGHT);
  motors[IDX_MOTOR_BACK_RIGHT] =
      new Motor(PIN_MOTOR_BACK_RIGHT, PIN_ENCODER_BACK_RIGHT);
  return 0;
}

Node::Node() {
  // TODO: Refactor status now that node is in a class
  // status = StatusManager::getInstance();

  // Initialize ROS
  allocator = rcl_get_default_allocator();
  // rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

  // Number of uRosHandles allocated in the executor (1 timer, 1 subscription,
  // 6 publisher)
  // TODO: Increment these memory handles
  const size_t uRosHandles = 12 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES;
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(
      rclc_executor_init(&executor, &support.context, uRosHandles, &allocator));

  // stdio_init_all(); // Called by uros
  init_watchdog();

  // -------------------------
  // Setup micro-ROS
  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);

  // Wait for agent successful ping for 2 minutes.
  RCCHECK(rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS));
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "deepdrive_micro_node", "", &support));

  status.set(Status::Connected);

  // Synchronize time with the agent
  rmw_uros_sync_session(5000);

  // LED Ring to show status
  led_ring.start();
  led_status_init();

  // Params server (currently not working)
  init_param_server();

  // Battery sensor & temperature sensor
  analog_sensors->init();
  init_battery();

  status.set(Status::Connecting);

  // Twist Subscriber
  RCCHECK(init_cmd_vel());

#ifdef IMU_ENABLED
  IMU imu = IMU();
#endif

  // Start Motors
  RCCHECK(init_motors());
}

void Node::spin() {
  // Launch the control loop on the other core
  start_control_loop();

  // Spin the executor on this core to pub/sub to ROS
  while (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1) == RCL_RET_OK)) {
#ifdef WATCHDOG_ENABLED
    watchdog_update();
#endif

    // TODO: Timer to monitor agent?
    // RCCHECK(rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS));
  }
}

// TODO: Maybe put this in a destructor
void Node::shutdown() {
  // Clean up resources
  // TODO: Cleanup the rest
  RCCHECK(rcl_publisher_fini(&publisher_motor, &node));
  RCCHECK(rcl_subscription_fini(&subscriber_motor, &node));
  RCCHECK(rcl_node_fini(&node));
}