#include <node.hpp>

static void timer_cb_main_loop(rcl_timer_t *timer, int64_t last_call_time) {
  Node::getInstance().spin_main_loop(timer, last_call_time);
}

int Node::init_main_loop() {
  return rclc_timer_init_default(&timer_main_loop, &support,
                                 RCL_MS_TO_NS(1000 / MAIN_LOOP_HZ),
                                 timer_cb_main_loop);
  // return rclc_timer_init_default(&timer_main_loop, &support,
  //                                 RCL_MS_TO_NS(1000),
  //                                 timer_cb_main_loop);
}

int Node::start_main_loop() {
  rcl_ret_t error_code = rclc_executor_add_timer(&executor, &timer_main_loop);
  if (error_code != RCL_RET_OK) {
    return error_code;
  }
  StatusManager::getInstance().set(Status::Connected);
  return 0;
}

void Node::spin_main_loop(rcl_timer_t *timer, int64_t last_call_time) {
  // TODO: Do some calculations in the other core and publish in this one
  static int core = get_core_num();

  pub_telemetry->set_core_start(core);

  pub_odom->publish();
  pub_telemetry->publish();
  pub_wheel_speed->publish();
  pub_battery_state->publish();
  pub_joint_state->publish();
  pub_imu->publish();

#ifdef SONAR_ENABLED
  pub_sonar_front->publish();
  pub_sonar_back->publish();
#endif

  pub_telemetry->set_core_stop(core);
}