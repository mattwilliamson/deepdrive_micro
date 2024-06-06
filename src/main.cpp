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
    sleep_ms(1000);
    watchdog_reboot(0, 0, 0);
    // exit(1);
  }
}

int64_t post_startup(alarm_id_t id, void* user_data) {
  Node::getInstance().startup_completed();
  return 0;
}

repeating_timer_t timer_led_ring;

// LED Ring animation for startup
static bool render_startup_led(repeating_timer_t* rt) {
  static LEDRing& led_ring = LEDRingStatus::getInstance().getLEDRing();
  static uint8_t t = 0;
  static int loop = 0;

  switch (loop) {
    case 0:
      led_ring.fadeW(0x99, t);
      break;
    case 1:
      led_ring.fadeR(0x99, t);
      break;
    case 2:
      led_ring.fadeW(0x99, t);
      break;
    // default:
      // led_ring.fill(0x99, 0x99, 0x99);
  }

  t++;
  if (t == 0) {
    loop++;
  }

  if (loop <= 3) {
    return true;
  }

  led_ring.fill(0x66, 0x66, 0x66);

  return false;
}

// LED Ring animation for startup
void startup_led() {
  LEDRing& led_ring = LEDRingStatus::getInstance().getLEDRing();
  led_ring.start();
  assert(add_repeating_timer_us(2 * 1000, render_startup_led, NULL, &timer_led_ring));
  sleep_ms(1000);
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

  startup_led();

  // Force node to initialize
  Node::getInstance();

  watchdog_update();

  // Wait for agent successful ping
  error_code = rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS);
  RCCHECK(error_code);

  // Synchronize time with the agent
  rmw_uros_sync_session(5000);

  // Reset motor pulse count in case there were any pulses counted before the control loop started
  assert(add_alarm_in_ms(STARTUP_DELAY, post_startup, NULL, true));

  // Start control loop and main loop
  Node& node_instance = Node::getInstance();
  node_instance.spin();

  return 0;
}