#include <node.hpp>

static void timer_cb_telemetry_loop(rcl_timer_t *timer, int64_t last_call_time) {
  Node::getInstance().spin_telemetry_loop(timer, last_call_time);
}

int Node::init_telemetry_loop() {
      return rclc_timer_init_default(&timer_telemetry_loop, &support,
                                  RCL_MS_TO_NS(1000 / TELEMETRY_LOOP_HZ),
                                  timer_cb_telemetry_loop);
}

int Node::start_telemetry_loop() {
  rcl_ret_t error_code = rclc_executor_add_timer(&executor, &timer_telemetry_loop);
  if (error_code != RCL_RET_OK) {
    return error_code;
  }
  return 0;
}

void Node::spin_telemetry_loop(rcl_timer_t* timer_main_loop, int64_t last_call_time) {
  const int timeout_ms = 1000;
  const int attempts = 1;

  rmw_ret_t error_code = rmw_uros_ping_agent(timeout_ms, attempts);

  // TODO: Probably want to check other criteria when disabling motors
  // TODO: Check if two motors on the same side have similar speeds

  if (error_code != RMW_RET_OK) {
    status.set(Status::Error);
    disable_motors();
  } else {
    // status.set(Status::Error);
    enable_motors();
  }

  publish_diagnostic();
}