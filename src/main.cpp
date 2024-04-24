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

// #define PICO_MALLOC_PANIC

// #include "imu/driver_mpu9250_interface.h"
// #include "imu/driver_mpu9250_dmp.h"

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

bool ledOn;
const uint LED_PIN = 14;

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  ledOn = !ledOn;

  if (timer != NULL) {
    printf("Timer callback executed. Last time %ld\n", last_call_time);
  }

  gpio_put(LED_PIN, ledOn);
}

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

int main() {
  // gpio_init(LED_PIN);
  // gpio_set_dir(LED_PIN, GPIO_OUT);

  stdio_init_all();

  rmw_uros_set_custom_transport(
      true,
      NULL,
      pico_serial_transport_open,
      pico_serial_transport_close,
      pico_serial_transport_write,
      pico_serial_transport_read);

  rcl_ret_t error_code;

  rcl_timer_t timer;
  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rclc_executor_t executor;

  allocator = rcl_get_default_allocator();

  // Wait for agent successful ping for 2 minutes.
  const int timeout_ms = 1000;
  const uint8_t attempts = 120;

  error_code = rmw_uros_ping_agent(timeout_ms, attempts);
  RCCHECK(error_code);

  Node& main_node = Node::getInstance();
  main_node.spin();

  // node.shutdown();

  // RCCHECK(1);

  return 0;
}