#include "sonar.h"

#ifdef SONAR_ENABLED

// Instantiate the sensors and publishers based on the config
HCSR04 sonarFrontRight(SONAR_TRIGGER_PIN_RIGHT, SONAR_ECHO_PIN_RIGHT);
HCSR04 sonarFrontLeft(SONAR_TRIGGER_PIN_LEFT, SONAR_ECHO_PIN_LEFT);

// Instantiate the publishers
UltrasonicRangePublisher* sonarRightPublisher;
UltrasonicRangePublisher* sonarLeftPublisher;

// Interrupt Service Routines for the sensors
void sonarFrontRightISR() {
    sonarFrontRight.echoISR();
}

void sonarFrontLeftISR() {
    sonarFrontLeft.echoISR();
}

void initSonar() {
    // Initialize the sensors
    sonarFrontRight.begin();
    sonarFrontLeft.begin();

    // Attach interrupts to the echo pins
    attachInterrupt(digitalPinToInterrupt(SONAR_ECHO_PIN_RIGHT), sonarFrontRightISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SONAR_ECHO_PIN_LEFT), sonarFrontLeftISR, CHANGE);

    // Initialize publishers
    sonarRightPublisher = new UltrasonicRangePublisher(
        SONAR_FRAME_RIGHT,
        SONAR_TOPIC_RANGE_RIGHT, 
        SONAR_TOPIC_SCAN_RIGHT, 
        sonarFrontRight, 
        SONAR_PUBLISH_RATE, 
        SONAR_PING_RATE, 
        &support, 
        &node, 
        &executor
    );
    sonarRightPublisher->init();

    sonarLeftPublisher = new UltrasonicRangePublisher(
        SONAR_FRAME_LEFT,
        SONAR_TOPIC_RANGE_LEFT, 
        SONAR_TOPIC_SCAN_LEFT, 
        sonarFrontLeft, 
        SONAR_PUBLISH_RATE, 
        SONAR_PING_RATE, 
        &support, 
        &node, 
        &executor
    );
    sonarLeftPublisher->init();
}

void updateSonar() {
    // Update the sensors and publish the data
    sonarRightPublisher->update();
    sonarLeftPublisher->update();
}

void sonarTask(void* pvParameters) {
    SerialDebug.println("Sonar task started");
    initSonar();
    SerialDebug.println("Sonar task inited");

    while (true) {
        // Update sonar readings and publish data
        updateSonar();

        // Delay to control the frequency of sonar updates
        // vTaskDelay(pdMS_TO_TICKS(1000 / SONAR_PUBLISH_RATE));  // Adjust the delay as needed
        delay(1);
    }
}

#endif // SONAR_ENABLED
