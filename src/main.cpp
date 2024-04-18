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

  //     error_code = rclc_support_init(&support, 0, NULL, &allocator);
  //     // RCCHECK(error_code);
  //     // error code is 1

  //     error_code = rclc_node_init_default(&node, "pico_node", "", &support);
  //     // RCCHECK(error_code);
  //     // error code is 1

  //     rclc_publisher_init_default(
  //         &publisher,
  //         &node,
  //         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //         "pico_publisher");

  //     rclc_timer_init_default(
  //         &timer,
  //         &support,
  //         RCL_MS_TO_NS(1000),
  //         timer_callback);

  //     rclc_executor_init(&executor, &support.context, 1, &allocator);
  //     rclc_executor_add_timer(&executor, &timer);

  // gpio_put(LED_PIN, 1);

  //     msg.data = 0;
  //     while (true)
  //     {
  //         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  //     }
  //     // return 0;

  // create init_options
  // rcl_ret_t error_code = rclc_support_init(&support, 0, NULL, &allocator);
  // RCCHECK(error_code);

  Node& main_node = Node::getInstance();
  main_node.spin();

  // TODO: Define topics in config

  // while (true) {
  //   #ifdef WATCHDOG_ENABLED
  //   watchdog_update();
  //   #endif

  //   RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  //   // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  //   // RCCHECK(rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS));
  // }

  // node.shutdown();

  // RCCHECK(1);

  return 0;
}