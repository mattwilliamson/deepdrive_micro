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
  if (!add_repeating_timer_us(-MICROSECONDS / CONTROL_LOOP_HZ, trigger_control, NULL, &timer_control)) {
    // printf("Failed to add control loop timer\r\n");
    return 1;
  }

  // Setup LED Ring animation loop timer
  if (!add_repeating_timer_us(-MICROSECONDS / LED_RING_HZ, trigger_led_ring, NULL, &timer_led_ring)) {
    // printf("Failed to add led ring timer\r\n");
    return 1;
  }

  return 0;
}

// Second core worker function
// LED Ring and control loop
void Node::spin_control_loop() {
  bool led_on = true;

  while (true) {
    core_start[1] = time_us_64();

    // Control loop for motors
    if (control_due) {
      control_due = false;

      // Read any motor encoder pulses
      motor_manager_->update_motor_outputs();

      pub_odom->calculate();
      pub_imu->calculate();
      pub_telemetry->calculate();
      pub_joint_state->calculate();

      // TODO: Do other processing here to make publishing on core0 as fast as
      // possible e.g. calculate odometry, publish sensor data, etc.

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
