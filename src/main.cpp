extern "C" {
#include "config.h"
}

#include <node.hpp>

#include "led_ring.hpp"
#include "motor.hpp"
#include "pubsub/pub_telemetry.hpp"
#include "status.hpp"

#ifdef IMU_ENABLED
#include "imu.hpp"
#endif

#define RCSOFTCHECK(fn)                                                              \
  {                                                                                  \
    rcl_ret_t temp_rc = fn;                                                          \
    if ((temp_rc != RCL_RET_OK)) {                                                   \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
    }                                                                                \
  }

void RCCHECK(rcl_ret_t error_code) {
  if (error_code != RCL_RET_OK) {
    printf("Failed status on line %d: %d. Exiting.\n", __LINE__, (int)error_code);
    exit(1);
  }
}


int main() {
  stdio_init_all();

  rmw_uros_set_custom_transport(
      true,
      NULL,
      pico_serial_transport_open,
      pico_serial_transport_close,
      pico_serial_transport_write,
      pico_serial_transport_read);

  rcl_timer_t timer;
  rcl_node_t node;
  rclc_support_t support;
  rclc_executor_t executor;
  rcl_allocator_t allocator = rcl_get_default_allocator();

  rcl_ret_t error_code;

  // Wait for agent successful ping for 2 minutes.
  error_code = rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS);
  RCCHECK(error_code);

  // Start control loop and main loop
  Node::getInstance().spin();

  return 0;
}