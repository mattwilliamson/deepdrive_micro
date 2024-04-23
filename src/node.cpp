#include "node.hpp"

int Node::init_motors() {
  // Setup 4 ESC brushless motor controllers
  motors.resize(MOTOR_COUNT);
  motors[IDX_MOTOR_FRONT_LEFT] = new Motor(MOTOR_LEFT, PIN_MOTOR_FRONT_LEFT, PIN_ENCODER_FRONT_LEFT);
  motors[IDX_MOTOR_BACK_LEFT] = new Motor(MOTOR_LEFT, PIN_MOTOR_BACK_LEFT, PIN_ENCODER_BACK_LEFT);
  motors[IDX_MOTOR_FRONT_RIGHT] = new Motor(MOTOR_RIGHT, PIN_MOTOR_FRONT_RIGHT, PIN_ENCODER_FRONT_RIGHT);
  motors[IDX_MOTOR_BACK_RIGHT] = new Motor(MOTOR_RIGHT, PIN_MOTOR_BACK_RIGHT, PIN_ENCODER_BACK_RIGHT);
  return 0;
}

void Node::disable_motors() {
  for (auto &motor : motors) {
    motor->disable();
  }
}

void Node::enable_motors() {
  for (auto &motor : motors) {
    motor->enable();
  }
}

Node::Node() {
  rcl_ret_t error_code;
  status.set(Status::Connecting);

  // TODO: Status booting, update after ping to connecting

  // TODO: Refactor status now that node is in a class
  // status = StatusManager::getInstance();

  // Initialize ROS
  allocator = rcl_get_default_allocator();

  // Wait for agent successful ping for 2 minutes.
  error_code = rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS);
  RCCHECK(error_code);
  if (error_code != RCL_RET_OK) {
    return;
  }
  // TODO: if spin is called after this fails, return an error

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "deepdrive_micro", "", &support);
  // rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

  init_main_loop();
  init_telemetry_loop();

  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, uRosHandles, &allocator);

  // stdio_init_all(); // Called by uros
  init_watchdog();

  // Synchronize time with the agent
  rmw_uros_sync_session(5000);

  // TODO: Error handling for all the init functions

  // LED Ring to show status
  led_ring.start();
  led_status_init();

  // Diagnostic publisher
  init_diagnostic();

  // Params server (currently not working)
  init_param_server();

  // Battery sensor & temperature sensor
  analog_sensors->init();
  init_battery();

  // IMU publisher
  status.setErrorString("Init IMU");
  publish_diagnostic();
  init_imu();

  // Motor publisher
  status.setErrorString("Init motor publisher");
  publish_diagnostic();
  init_motor_pub();

  // Odom publisher
  status.setErrorString("Init odom");
  publish_diagnostic();
  init_odom();

  // Joint State publisher
  status.setErrorString("Init joint state");
  publish_diagnostic();
  init_joint_state();

  status.set(Status::Connected);

  // Twist Subscriber
  status.setErrorString("Init cmd vel");
  publish_diagnostic();
  RCCHECK(init_cmd_vel());

  // Start Motors
  status.setErrorString("Init Motors");
  publish_diagnostic();
  RCCHECK(init_motors());

  status.set(Status::Connected);
}

void Node::spin() {
  rcl_ret_t error_code;

  // Launch the control loop on the other core
  start_control_loop();

  // Main pub/sub loop is on a timer inside rclc_executor
  error_code = start_main_loop();
  RCCHECK(error_code);

  // Telemtry loop is on a timer inside rclc_executor
  error_code = start_telemetry_loop();
  RCCHECK(error_code);

  // Spin the executor on this core to pub/sub to ROS
  while (true) {
#ifdef WATCHDOG_ENABLED
    watchdog_update();
#endif

    // rclc_executor_spin(&executor);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    // TODO: Timer to monitor agent?
    // RCCHECK(rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS));
  }
}

// TODO: Maybe put this in a destructor
void Node::shutdown() {
  // Clean up resources
  // TODO: Cleanup the rest
  // RCCHECK(rcl_publisher_fini(&publisher_motor_speed, &node));
  RCCHECK(rcl_subscription_fini(&subscriber_motor, &node));
  RCCHECK(rcl_node_fini(&node));
}