#include "node.hpp"

Node::Node() {
  rcl_ret_t error_code;
  status.set(Status::Connecting);

  // TODO: Status booting, update after ping to connecting

  // TODO: Refactor status now that node is in a class
  // status = StatusManager::getInstance();

  // Initialize ROS
  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "deepdrive_micro", "", &support);
  // rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

  init_main_loop();

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

  // Params server (currently not working)
  // init_param_server();

  // Battery sensor & temperature sensor
  analog_sensors = new AnalogSensors();

  motor_manager_ = new MotorManager();

  // IMU publisher
  pub_imu = new PubImu(&node, &support, &allocator);
  pub_joint_state = new PubJointState(&node, &support, &allocator, motor_manager_);
  pub_telemetry = new PubTelemetry(&node, &support, &allocator, motor_manager_);
  pub_odom = new PubOdom(&node, &support, &allocator, motor_manager_);
  pub_battery_state = new PubBatteryState(&node, &support, &allocator, analog_sensors);

  sub_cmd_vel = new SubCmdVel(&node, &support, &allocator, &executor, motor_manager_);

  status.set(Status::Connected);
}

void Node::spin() {
  rcl_ret_t error_code;

  // Launch the control loop on the other core
  start_control_loop();

  // Main pub/sub loop is on a timer inside rclc_executor
  error_code = start_main_loop();
  RCCHECK(error_code);

  // Spin the executor on this core to pub/sub to ROS
  while (true) {
#ifdef WATCHDOG_ENABLED
    watchdog_update();
#endif
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  }
}

// TODO: Maybe put this in a destructor
Node::~Node() {
  RCCHECK(rcl_node_fini(&node));

  // TODO: Make this memory handled better
  cancel_repeating_timer(&timer_control);
  cancel_repeating_timer(&timer_led_ring);

  delete motor_manager_;
  delete analog_sensors;

  delete pub_imu;
  delete pub_joint_state;
  delete pub_telemetry;
  delete pub_odom;
  delete pub_battery_state;

  delete sub_cmd_vel;

}