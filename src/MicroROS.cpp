#include "MicroROS.h"

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

RosoutLogger *logger;

uint32_t last_message_time = 0;
uint32_t last_ping_success_time = 0;
uint32_t ping_failures = 0;
bool connected = false;

bool micro_ros_started = false;


void setupMicroROS();

// Error handle loop
void error_loop();

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      SerialDebug.println("error code: " + String(temp_rc));                       \
      error_loop();                                                            \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }
// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc !=
// RCL_RET_OK)){printf("Failed status on line %d: %d.
// Aborting.\n",__LINE__,(int)temp_rc); return 1;}} #define RCSOFTCHECK(fn) {
// rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on
// line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


// Error handle loop
void error_loop() {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("Error");
  }
}

void vTaskMicroROS(void *pvParameters) {
  
  SerialDebug.println("setupMicroROS waiting for serial");

  while (!Serial); // Wait for serial port to connect (needed for some boards)

    // xTaskCreate(vTaskBuzzer, "BuzzerTask", 10000, NULL, 1, NULL);
  SerialDebug.println("setupMicroROS setting transport");

  set_microros_serial_transports(Serial);
  delay(1000);

  // Initialize micro ros

  SerialDebug.println("setupMicroROS alloc memory");

  // Alloc memory
  allocator = rcl_get_default_allocator();
  SerialDebug.println("setupMicroROS support init");
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  SerialDebug.println("error code: " + String(ret));

  SerialDebug.println("setupMicroROS init node");
  RCCHECK(rclc_node_init_default(&node, "deepdrive_micro", "", &support));

  // Executor setup
  SerialDebug.println("setupMicroROS executor init");
  RCCHECK(rclc_executor_init(&executor, &support.context, 16, &allocator));

  logger = new RosoutLogger(&node, &support);

  SerialDebug.println("setupMicroROS done");
  micro_ros_started = true;

  while(true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    yield();
  }
}

void vTaskPing(void *pvParameters) {
  // last_message_time = 0;
  // ping_failures = 0;

  // while (true) {
    // If we don't get messages and the agent is not available, stop the motors
    uint32_t elapsed = millis() - last_message_time;

    if (elapsed > MESSAGE_RECEIVE_TIMEOUT) {
      // We lost communication with the micro ros agent. Stop the motors.
    //   frontMotorController->setTargetVelocity(0);
    //   backMotorController->setTargetVelocity(0);

      // Serial.println("Error: No message received for " + String(elapsed)
      // +"ms"); Serial.println("ping_failures: " + String(ping_failures));
      SerialDebug.println("vTaskPing pinging agent");

      if (rmw_uros_ping_agent(UROS_TIMEOUT_PERIODIC, 1) != RMW_RET_OK) {
        // Serial.println("ping failed");

        connected = false;
        ping_failures++;

        if (ping_failures >= AGENT_PING_ATTEMPTS_REBOOT) {
          // Reboot the board
          Serial.println(
              "Critical: Agent not available after retries. Rebooting");
          delay(1000);
          abort();
        }
      } else {
        // Initial ping successful to agent
        if (!connected) {
          last_ping_success_time = millis();
          connected = true;
          ping_failures = 0;
          // Serial.println("initial connection");

          // Synchronize time with the agent
          // rmw_uros_sync_session(100);

          // backMotorController->playSuccess();
          // frontMotorController->playSuccess();
        }
      }
    }

  //   // vTaskDelay(xDelay);
  //   vTaskDelay(pdMS_TO_TICKS(10));
  // }
}

// void vTaskMicroROS(void *pvParameters) {
//   SerialDebug.println("vTaskMicroROS starting");
// //   const TickType_t xDelay = MOTOR_LOOP_PERIOD * portTICK_PERIOD_MS / 1000;

//   // Serial.println("Wait for ping thread to succeed");

//   // Serial.println("initializing micro ros inside vTaskMicroROS");


//   // Wait for ping task to succeed
//   while (!connected) {
//     SerialDebug.println("vTaskMicroROS waiting for connection");
//     vTaskDelay(pdMS_TO_TICKS(100));
//   }

//   while (true) {
//     // RCCHECK(rclc_executor_spin_some(&executor));
//     // TODO: Should this go smaller? Will it block the kernel?
//     rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
//     // vTaskDelay(MOTOR_LOOP_PERIOD * portTICK_PERIOD_MS / 1000);
//     vTaskDelay(pdMS_TO_TICKS(1));
//   }
// }