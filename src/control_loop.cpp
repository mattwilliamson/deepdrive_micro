#include <node.hpp>

static void spin_control_loop_callback() {
  Node& node = Node::getInstance();
  node.spin_control_loop();
}

static bool trigger_control(repeating_timer_t* rt) {
  Node::getInstance().control_due = true;
  return true;
}

static bool trigger_led_ring(repeating_timer_t* rt) {
  Node::getInstance().render_led_ring = true;
  return true;
}

int Node::start_control_loop() {
  // Parallel processing core - RGB LED Ring and control loop
  multicore_launch_core1(spin_control_loop_callback);

  // Setup control loop timer
  assert(add_repeating_timer_us(-MICROSECONDS / CONTROL_LOOP_HZ, trigger_control, NULL, &timer_control));

  // TODO: move this to separate class
  // Setup LED Ring animation loop timer
  assert(add_repeating_timer_us(-MICROSECONDS / LED_RING_HZ, trigger_led_ring, NULL, &timer_led_ring));

  return 0;
}

// Second core worker function
// LED Ring and control loop
void Node::spin_control_loop() {
  bool led_on = true;
  static int core = get_core_num();

  while (true) {
    pub_telemetry->set_core_start(core);

    // Control loop for motors
    if (control_due) {
      control_due = false;

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
    pub_sonar->calculate();

#ifndef LED_RING_ENABLED
    led_ring.off();
#else
    // LED Ring Loop
    if (render_led_ring) {
      render_led_ring = false;
      led_on = !led_on;
      led_ring.renderStatus(status.get());
      led_status_set(led_on);
    }
#endif

    int64_t lastCmdVel = sub_cmd_vel->getLastMessage();
    if (lastCmdVel != 0 && rmw_uros_epoch_nanos() - sub_cmd_vel->getLastMessage() > CMD_VEL_TIMEOUT) {
      motor_manager_->disable_motors();
      StatusManager::getInstance().set(Status::Error);
      sub_cmd_vel->resetLastMessage();
    }

    pub_telemetry->set_core_stop(core);

    // Sit tight until we have more work to do
    // tight_loop_contents();
    // sleep_ms(1);
  }
}
