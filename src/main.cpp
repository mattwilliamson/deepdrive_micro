#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

#include "config.h"
#include "MicroROS.h"
#include "Buzzer.hpp"

#ifdef SONAR_ENABLED
#include <PubSonar.hpp>
#include <HC_SR04.h>

HC_SR04<SONAR_ECHO_PIN_RIGHT> sonar_right(SONAR_TRIGGER_PIN_RIGHT);
HC_SR04<SONAR_ECHO_PIN_LEFT> sonar_left(SONAR_TRIGGER_PIN_LEFT);
PubSonar *pub_sonar_right;
PubSonar *pub_sonar_left;

void setupSonar() {

  
  pub_sonar_right = new PubSonar(&node, &support, &allocator,
                                 SONAR_TRIGGER_PIN_RIGHT, static_cast<HC_SR04_BASE *>(&sonar_right), SONAR_PUBLISH_RATE,
                                 SONAR_TOPIC_RIGHT, SONAR_FRAME_RIGHT);
  pub_sonar_left = new PubSonar(&node, &support, &allocator,
                                SONAR_TRIGGER_PIN_LEFT, static_cast<HC_SR04_BASE *>(&sonar_left), SONAR_PUBLISH_RATE,
                                SONAR_TOPIC_LEFT, SONAR_FRAME_LEFT);
                              
}
#endif

void vTaskCoreMonitor(void *pvParameters) {
  unsigned int taskIndex = (unsigned int)pvParameters;
  volatile unsigned int core = get_core_num();
  while (true) {
    Serial.println("CoreMonitor" + String(taskIndex) + "Core: " + String(core) + " running out of " + String(configNUMBER_OF_CORES));
    // Serial.println("CoreMonitor Core %d: " + String(get_core_num()) + " running out of " + String(NUM_CORES));
    delay(100);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Core 0
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
    
  setupMicroROS();
  // xTaskCreate(vTaskBuzzer, "BuzzerTask", 10000, NULL, 1, NULL);

  // Setup the ROS logger
  // logger = new RosoutLogger(&node, &support);
  // logger->println("ROS logger initialized");

  xTaskCreate(vTaskPing, "PingTask", 10000, NULL, 4, NULL);
  xTaskCreate(vTaskMicroROS, "MicroROSTask", 10000, NULL, 8, NULL);
  // Serial.println("done creating micro ros task");
  
}

// Core 0
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
  // Serial.println("main loop core 0");
}

// Core 1
void setup1() {

  // TODO: Buzzer config
  setupBuzzer();

  #ifdef SONAR_ENABLED
  setupSonar();
  #endif
}

// Core 1
void loop1() {
  playBuzzer();
  vTaskDelay(pdMS_TO_TICKS(1));
  // Serial.println("main loop core 1");
}