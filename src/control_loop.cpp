#include <node.hpp>

static void spin_control_loop_callback() {
  Node& node = Node::getInstance();
  node.spin_control_loop();
}

static bool trigger_control(repeating_timer_t* rt) {
  Node::getInstance().motors_due = true;
  return true;
}

static bool trigger_led_ring(repeating_timer_t* rt) {
#ifndef LED_RING_ENABLED
  led_ring.off();
  return false;
#else
  // LED Ring Loop
    LEDRingStatus::getInstance().getLEDRing().renderStatus(StatusManager::getInstance().get());

  // Toggle the LED
    bool led_on = true;
    led_on = !led_on;
    led_status_set(led_on);
#endif

  return true;
}

void Node::start_led_ring() {
  // TODO: move this to separate class
  // Setup LED Ring animation loop timer
  assert(add_repeating_timer_us(-MICROSECONDS / LED_RING_HZ, trigger_led_ring, NULL, &timer_led_ring));

}

int Node::start_control_loop() {
  // Parallel processing core - RGB LED Ring and control loop
  multicore_launch_core1(spin_control_loop_callback);

  // Setup control loop timer
  assert(add_repeating_timer_us(-MICROSECONDS / CONTROL_LOOP_HZ, trigger_control, NULL, &timer_control));

  return 0;
}

// Second core worker function
// LED Ring and control loop
void Node::spin_control_loop() {
  static int core = get_core_num();

  while (true) {
    pub_telemetry->set_core_start(core);

    // Control loop for motors
    if (motors_due) {
      motors_due = false;

      // Read any motor encoder pulses
      motor_manager_->update_motor_outputs();
    }

    // Publish any due messages
    pub_odom->calculate();
    pub_imu->calculate();
    pub_telemetry->calculate();
    pub_joint_state->calculate();
    pub_wheel_speed->calculate();
    pub_battery_state->calculate();

#ifdef SONAR_ENABLED
    pub_sonar_front->calculate();
    pub_sonar_back->calculate();
#endif

    check_cmd_vel_timeout();

    pub_telemetry->set_core_stop(core);

    // Sit tight until we have more work to do
    // tight_loop_contents();
    // sleep_ms(1);
  }
}

void Node::check_cmd_vel_timeout() {
  // Check for timeout on cmd_vel
  int64_t lastCmdVel = sub_cmd_vel->getLastMessage();
  if (lastCmdVel != 0) {
    int64_t now =  rmw_uros_epoch_nanos();
    int64_t elapsed = now - lastCmdVel;

    if (elapsed > CMD_VEL_TIMEOUT_DISABLE) {
      motor_manager_->disable_motors();
      StatusManager::getInstance().set(Status::Error);
      StatusManager::getInstance().setErrorString("CMD_VEL_TIMEOUT");
      sub_cmd_vel->resetLastMessage();
    } else if (elapsed > CMD_VEL_TIMEOUT) {
      motor_manager_->stop_motors();
      sub_cmd_vel->resetLastMessage();
    } else {
      // motor_manager_->enable_motors();
    }
  } else {
    motor_manager_->stop_motors();
  }
}