extern "C" {
#include "config.h"
}

#include <node.hpp>

#include "led_ring.hpp"
#include "motor.hpp"
#include "pub_diagnostic.hpp"
#include "status.hpp"

#ifdef IMU_ENABLED
#include "imu.hpp"
#endif

// #include "imu/driver_mpu9250_interface.h"
// #include "imu/driver_mpu9250_dmp.h"

// void timer_cb_led_ring(rcl_timer_t *timer, int64_t last_call_time) {
//   static uint32_t t = 0;
//   led_ring_pattern_snakes(LED_RING_NUM_PIXELS, t);
//   t++;
// }

int main() {
  Node& node = Node::getInstance();
  node.spin();

  // TODO: Define topics in config

  // while (true) {
  //   #ifdef WATCHDOG_ENABLED
  //   watchdog_update();
  //   #endif

  //   RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  //   // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  //   // RCCHECK(rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS));
  // }

  node.shutdown();

  return 0;
}