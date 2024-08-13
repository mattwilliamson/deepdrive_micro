#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

#include "Buzzer.h"
#include "MicroROS.h"
#include "StatusLED.h"
#include "StatusManager.h"
#include "config.h"
#include "imu.h"
#include "sonar.h"

StatusLED statusLED(LED_RING_NUM_PIXELS, LED_RING_PIN);

void vTaskCoreMonitor(void *pvParameters) {
  unsigned int taskIndex = (unsigned int)pvParameters;
  volatile unsigned int core = get_core_num();
  while (true) {
    SerialDebug.println("CoreMonitor" + String(taskIndex) + "Core: " + String(core) + " running out of " + String(configNUMBER_OF_CORES));
    // Serial.println("CoreMonitor Core %d: " + String(get_core_num()) + " running out of " + String(NUM_CORES));
    delay(5000);
    // vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void vTaskStatusLED(void *pvParameters) {
  statusLED.begin();

  while (true) {
    delay(10);
    statusLED.update();
  }
}

// Core 0
void setup() {
  StatusManager::getInstance().setStatus(CONNECTING);

  Serial.begin(SERIAL_BAUD_RATE);
  SerialDebug.begin(SERIAL_BAUD_RATE);

  xTaskCreate(vTaskStatusLED, "TaskStatusLED", 10000, NULL, 0, NULL);

  // Serial.println("Starting setup");
  delay(1000);
  SerialDebug.println("s1 Starting setup");

  SerialDebug.println("setup");
  // xTaskCreate(vTaskCoreMonitor, "vTaskCoreMonitor", 10000, (void *)0, 4, NULL);

  // setupMicroROS();
  // micro_ros_started = true;
  xTaskCreate(vTaskMicroROS, "MicroROSTask", 10000, NULL, 1, NULL);
  // xTaskCreate(vTaskBuzzer, "BuzzerTask", 10000, NULL, 1, NULL);

  // Serial1.println("Checking ROS Logger");
  // logger->Debug("ROS logger initialized for deepdrive-micro");
  // logger->Info("ROS logger initialized for deepdrive-micro");
  // Serial1.println("Checking ROS Logger done");

  // Setup the ROS logger
  // logger = new RosoutLogger(&node, &support);
  // logger->println("ROS logger initialized");
  // Serial.println("Creating tasks");
  SerialDebug.println("s1 Creating tasks\n\n\n\n");
  xTaskCreate(vTaskPing, "PingTask", 10000, NULL, 4, NULL);
  // xTaskCreate(vTaskMicroROS, "MicroROSTask", 10000, NULL, 8, NULL);
  // Serial.println("done creating micro ros task");
  SerialDebug.println("s1 done creating micro ros task\n\n\n");

#ifdef SONAR_ENABLED
  SerialDebug.println("Creating sonar task");
  xTaskCreate(sonarTask, "SonarTask", 10000, NULL, 1, NULL);
  SerialDebug.println("Done Creating sonar task");
#endif

  delay(500);

#ifdef IMU_ENABLED
  SerialDebug.println("starting IMU task");
  xTaskCreate(imuTask, "IMUTask", 10000, NULL, 1, NULL);
  SerialDebug.println("imu task started\n\n\n\n");
#endif
}

// TODO: Update in one core, publish in the other

// Core 0
void loop() {
  // vTaskDelay(pdMS_TO_TICKS(1000));
  // Serial.println("main loop core 0");
  // SerialDebug.println("s1 main loop core 0");
  // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  // vTaskDelay(MOTOR_LOOP_PERIOD * portTICK_PERIOD_MS / 1000);
  // vTaskDelay(pdMS_TO_TICKS(1));
  // yield();
  delay(1);
}

// Core 1
void setup1() {
  // SerialDebug.begin(SERIAL_BAUD_RATE);
  // Serial.println("Starting setup1");
  delay(1000);

  // SerialDebug.println("setup1");
  // xTaskCreate(vTaskCoreMonitor, "vTaskCoreMonitor", 10000, (void *)1, 4, NULL);

  SerialDebug.println("setup1");

  while (!micro_ros_started) {
    // SerialDebug.println("setup1 waiting for ROS");
    delay(1000);
  }

  // #ifdef IMU_ENABLED
  // SerialDebug.println("starting IMU task");
  //   xTaskCreate(imuTask, "IMUTask", 10000, NULL, 1, NULL);
  //   SerialDebug.println("imu task started");
  // #endif

  // #ifdef SONAR_ENABLED
  //     xTaskCreate(sonarTask, "SonarTask", 10000, NULL, 1, NULL);
  // #endif

  // TODO: Buzzer config
  // setupBuzzer();
}

// Core 1
void loop1() {
  // playBuzzer();
  // vTaskDelay(pdMS_TO_TICKS(1));
  // Serial.println("main loop core 1");
  // SerialDebug.println("s1 main loop core 1");
  delay(1000);

  yield();
}