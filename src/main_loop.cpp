#include <node.hpp>

static void timer_cb_main_loop(rcl_timer_t *timer, int64_t last_call_time) {
  Node::getInstance().spin_main_loop(timer, last_call_time);
}

int Node::init_main_loop() {
  return rclc_timer_init_default(&timer_main_loop, &support,
                                  RCL_MS_TO_NS(1.0 / MAIN_LOOP_HZ * 1000),
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
  return 0;
}

void Node::spin_main_loop(rcl_timer_t *timer, int64_t last_call_time) {
  // TODO: Do some calculations in the other core and publish in this one
  core_start[0] = time_us_64();

  publish_motor();

  // if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
  //   // Lost connection to agent. Stop motors.
  //   for(auto &motor : motors) {
  //     motor->setSpeed(0);
  //   }
  //   status.set(Status::Error);
  //   printf("micro-ROS agent has stopped. Exiting...\r\n");
  //   sleep_ms(1000);
  //   exit(1);
  // } else {
  //   printf("Agent is still up!\r\n\r\n");
  // }

  // TODO: Move some of these to config

  publish_battery();

  // 13,470us to publish_joint_state
  publish_joint_state();

  // 630us to publish_imu
  publish_imu();

  publish_odom();

  core_elapsed[0] = time_us_64() - core_start[0];

  publish_diagnostic();
}