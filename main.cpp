#include "main.h"

// const uint PIN_MOTOR_A = 0;
// const uint PIN_MOTOR_B = 1;
// const uint PIN_PULSE_A = 2;
uint8_t on = 0;
int8_t direction = 1;
uint pulses = 0;
int speed = 65000;
const uint max_speed = 65000;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msgOut;

// Set up subscriber
rcl_subscription_t subscriber;
std_msgs__msg__Int32 cmd;

LEDRing ledRing = LEDRing(LED_RING_PIN, LED_RING_PIO, LED_RING_NUM_PIXELS);
StatusManager& status = StatusManager::getInstance();
std::vector<Motor> motors;


void core1_entry() {
  // Run the led ring animation loop on the other core in parallel
  // multicore_fifo_push_blocking(FLAG_VALUE);

  // Wait for first status push
  // uint32_t g = multicore_fifo_pop_blocking();
  // uint32_t g = (uint32_t)Status::Init;
  bool led_on = true;

  #ifndef LED_RING_ENABLED
  ledRing.off();
  return;
  #endif

  while (true) {
    led_on = !led_on;
    ledRing.renderStatus(status.get());
    led_status_set(led_on);
    sleep_ms(LED_RING_DELAY_MS);
    // sleep_ms(100);

    // Wait for next status push
    // Might not actually need IPC here
    // if (multicore_fifo_pop_timeout_us(0, &g)) {
    //   ledRing.resetAnimation();
    // }
  }
}

void blink_error() {
  led_status_blink(1000, 300, 100);
}

void timer_cb_general(rcl_timer_t *timer, int64_t last_call_time) {
//   // Blink to show activity
//   led_on = !led_on;
// #if LIB_PICO_CYW43_ARCH
//   if (led_on) {
//     cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
//   } else {
//     cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
//   }
// #endif
static bool led_status = false;
led_status = !led_status;
// led_status_set(led_status);

  std_msgs__msg__Int32 newMsgOut;
  // newMsgOut.data = pulses;
  for (auto& motor : motors) {
    newMsgOut.data = motor.getPulses();
    if (newMsgOut.data != 0) {
      break;
    }
  }
  // msgOut.data = globalTicks;
  // msg.data = motors[3].getPulses();
  if (RCL_RET_OK != rcl_publish(&publisher, &newMsgOut, NULL)) {
    blink_error();
  }
  // pulses = 0;

  // on = !on;
  // gpio_put(PIN_MOTOR_A, 0);
  // // gpio_put(PIN_MOTOR_B, on);
  // // speed is max uint16_t or INT_MAX?
  // // Set PWM level to speed
  // if (speed > max_speed) {
  //   speed = max_speed;
  // }
  // pwm_set_gpio_level(PIN_MOTOR_B, speed);

  // if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
  //   // Lost connection to agent. Stop motors.
  //   pwm_set_gpio_level(PIN_MOTOR_B, 0);
  //   printf("micro-ROS agent has stopped. Exiting...\n");
  //   exit(1);
  // } else {
  //   printf("Agent is still up!\n\n");
  // }
}

// void timer_cb_led_ring(rcl_timer_t *timer, int64_t last_call_time) {
//   static uint32_t t = 0;
//   led_ring_pattern_snakes(LED_RING_NUM_PIXELS, t);
//   t++;
// }

// Subscriber callback
void subscription_callback(const void *msgin) {
  // Cast received message to used type
  const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;
  // speed = m->data;
  status.set(Status::Active);
  // for each motor, set speed
  for (auto& motor : motors) {
    motor.setSpeed(m->data);
  }
}

int main() {
  ledRing.start();
  led_status_init();

  // Add a Motor instance for all 4 motors PIN_MOTOR_FRONT_LEFT and so on
  motors.emplace_back(PIN_MOTOR_FRONT_LEFT, PIN_ENCODER_FRONT_LEFT);
  motors.emplace_back(PIN_MOTOR_FRONT_RIGHT, PIN_ENCODER_FRONT_RIGHT);
  motors.emplace_back(PIN_MOTOR_BACK_LEFT, PIN_ENCODER_BACK_LEFT);
  motors.emplace_back(PIN_MOTOR_BACK_RIGHT, PIN_ENCODER_BACK_RIGHT);

  // Arm motors
  // for (auto& motor : motors) {
  //   motor.setSpeed(-250);
  // }
  // sleep_ms(200);
  // for (auto& motor : motors) {
  //   motor.setSpeed(0);
  // }
  // sleep_ms(1000);
  

  // Wait for motors to start
  // sleep_ms(1000);

  // for (int i = 0; i <= 32000; i+=10) {
  //   for (auto& motor : motors) {
  //     motor.setSpeed(i);
  //   }
  //   ledRing.fill(0, 0, i>>8);
  //   sleep_ms(1);
  // }

  // for (int i = 32000; i >= 0 ; i-=10) {
  //     for (auto& motor : motors) {
  //       motor.setSpeed(i); 
  //     }
  //     ledRing.fill(0, 0, i>>8);
  //     sleep_ms(1);
  // }

  // for (int i = 0; i >= -32000; i-=10) {
  //   for (auto& motor : motors) {
  //     motor.setSpeed(i);
  //   }
  //   ledRing.fill(i>>8, 0, 0);
  //   sleep_ms(1);
  // }

  // for (int i = -32000; i <= 0 ; i+=10) {
  //     for (auto& motor : motors) {
  //       motor.setSpeed(i); 
  //     }
  //     ledRing.fill(i>>8, 0, 0);
  //     sleep_ms(1);
  // }

  // sleep_ms(100000);
  // Parallel processing core
  multicore_launch_core1(core1_entry);

  // sleep_ms(1000);
  status.set(Status::Connecting);
  // sleep_ms(25000);
  // status.set(Status::Error);
  // sleep_ms(5000);
  // status.set(Status::Success);
  // sleep_ms(2000);
  // status.set(Status::Active);
  // sleep_ms(2000);
  // status.set(Status::Connecting);

  #ifdef WATCHDOG_ENABLED
  // We rebooted because we got stuck or something
  if (watchdog_caused_reboot()) {
    // printf("Rebooted by Watchdog!\n");
    status.setstatus(STATUS_REBOOTED);
    sleep_ms(10000);
  }

  // Enable the watchdog, requiring the watchdog to be updated every 100ms or the chip will reboot
  // second arg is pause on debug which means the watchdog will pause when stepping through code
  watchdog_enable(1000, false);
  #endif

  // led_status_set(true);
  // sleep_ms(1000);
  // led_status_blink(30, 100, 100);
  // led_status_blink(100, 10, 100);

  // msgOut.data = 0;

  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);

  // gpio_init(PIN_MOTOR_A);
  // gpio_set_dir(PIN_MOTOR_A, GPIO_OUT);

  // gpio_init(PIN_MOTOR_B);
  // gpio_set_dir(PIN_MOTOR_B, GPIO_OUT);

  // Pulse counter (rotary encoder) input
  // gpio_set_irq_enabled_with_callback(PIN_PULSE_A,
  //                                    GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
  //                                    true, &gpio_callback);

  // PWM Out for motor control
  // uint slice_num = pwm_gpio_to_slice_num(PIN_MOTOR_B);
  // gpio_set_function(PIN_MOTOR_B, GPIO_FUNC_PWM);
  // pwm_set_wrap(slice_num, max_speed); // default clock is 125MHz (8ns)
  // pwm_set_enabled(slice_num, true);
  // pwm_set_gpio_level(PIN_MOTOR_B, 0);

  // ROS2 Node
  rcl_timer_t timer;
  // rcl_timer_t timer_led_ring;
  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rclc_executor_t executor;

  // Number of handles allowed in the executor (2 timer, 1 subscription)
  // Make sure to update as more are added!
  const size_t handles = 2;

  allocator = rcl_get_default_allocator();
  executor = rclc_executor_get_zero_initialized_executor();

  // Wait for agent successful ping for 2 minutes.
  // Connect to agent and init node
  // Check once and fail quickly if agent is not reachable to show error
  RCSOFTCHECK(rmw_uros_ping_agent(30, 1));

  // Then retry until agent is reachable and fail hard
  RCCHECK(rmw_uros_ping_agent(UROS_TIMEOUT, UROS_ATTEMPTS));

  status.set(Status::Connected);

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "deepdrive_micro_node", "", &support));

  // Publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "deepdrive_micro/pulses"));

  // Timer
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100),
                                  timer_cb_general));

// LED Ring Timer
  // RCCHECK(rclc_timer_init_default(&timer_led_ring, &support, RCL_MS_TO_NS(100),
                                  // timer_cb_led_ring));

  RCCHECK(rclc_executor_init(&executor, &support.context, handles, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer_led_ring));

  // Subscriber
  // ros2 topic pub --once deepdrive_micro/cmd std_msgs/msg/Int32 "{data:
  // 65000}"
  RCCHECK(rclc_subscription_init_best_effort(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "deepdrive_micro/cmd"));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd,
                                         &subscription_callback, ON_NEW_DATA));

  while (true) {
    #ifdef WATCHDOG_ENABLED
    watchdog_update();
    #endif

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
  // rclc_executor_spin(&executor);

  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  RCCHECK(rcl_node_fini(&node));

  return 0;
}