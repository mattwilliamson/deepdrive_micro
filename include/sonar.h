#ifndef SONAR_H
#define SONAR_H

#include "config.h"
#include "HCSR04.h"
#include "UltrasonicRangePublisher.h"
#include <Arduino.h>
#include <FreeRTOS.h>

#ifdef SONAR_ENABLED

// Forward declaration of ISR functions
void sonarFrontRightISR();
void sonarFrontLeftISR();

/**
 * @brief Initializes the sonar sensors, publishers, and tasks.
 */
void initSonar();

/**
 * @brief Updates the sonar sensors and publishes the data.
 */
void updateSonar();

/**
 * @brief Task function for FreeRTOS to handle sonar sensor reading and publishing.
 * 
 * @param pvParameters Pointer to the task parameters (unused).
 */
void sonarTask(void* pvParameters);

#endif // SONAR_ENABLED

#endif // SONAR_H
