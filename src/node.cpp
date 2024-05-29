#include "node.hpp"

LEDRing led_ring = LEDRing(LED_RING_PIN, LED_RING_PIO, LED_RING_NUM_PIXELS);

Node::Node() {
  rcl_ret_t error_code;

  // Buzzer for audio feedback
  buzzer = new Buzzer(PIN_BUZZER);
  // buzzer->stop();

#ifdef WATCHDOG_ENABLED
  if (watchdog_caused_reboot()) {
    // printf("Rebooted by Watchdog!\r\n");
    status.set(Status::Rebooted);
    // buzzer->stop();
    buzzer->playTune(Buzzer::Tune::REBOOTED);
    sleep_ms(10000);
    buzzer->stop();
  }
  // Enable the watchdog, requiring the watchdog to be updated every 10000ms or the chip will reboot
  watchdog_enable(WATCHDOG_TIMEOUT, false);
#endif

  buzzer->playTune(Buzzer::Tune::STARTUP);

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
  // init_watchdog();

  // TODO: Error handling for all the init functions

  // LED Ring to show status
  led_ring.start();
  led_status_init();
  start_led_ring();

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
  pub_battery_state = new PubBatteryState(&node, &support, &allocator, analog_sensors, buzzer);
  pub_wheel_speed = new PubWheelSpeed(&node, &support, &allocator, motor_manager_);

#ifdef SONAR_ENABLED
  pub_sonar_front = new PubSonar(&node, &support, &allocator,
                                 SONAR_TRIGGER_PIN_FRONT, SONAR_ECHO_PIN_FRONT, SONAR_PUBLISH_RATE,
                                 SONAR_TOPIC_FRONT, SONAR_FRAME_FRONT);
  pub_sonar_back = new PubSonar(&node, &support, &allocator,
                                SONAR_TRIGGER_PIN_BACK, SONAR_ECHO_PIN_BACK, SONAR_PUBLISH_RATE,
                                SONAR_TOPIC_BACK, SONAR_FRAME_BACK);
#endif

  sub_cmd_vel = new SubCmdVel(&node, &support, &allocator, &executor, motor_manager_);
  sub_wheel_speed = new SubWheelSpeed(&node, &support, &allocator, &executor, motor_manager_);
}

void Node::startup_completed() {
  status.set(Status::Connected);
  buzzer->playTune(Buzzer::Tune::POSITIVE);
  motor_manager_->reset_motor_pulse_count();

  // Motors will be enabled when we receive a cmd_vel call
  // motor_manager_->enable_motors();
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
  delete buzzer;

  delete pub_imu;
  delete pub_joint_state;
  delete pub_wheel_speed;
  delete pub_telemetry;
  delete pub_odom;
  delete pub_battery_state;

#ifdef SONAR_ENABLED
  delete pub_sonar_front;
  delete pub_sonar_back;
#endif

  delete sub_cmd_vel;
  delete sub_wheel_speed;
}