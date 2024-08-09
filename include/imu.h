#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include "MicroROS.h"
#include "ICM_20948.h"
#include "PubICM20948.h"
#include "config.h"

#ifdef IMU_ENABLED

// Make sure we have DMP support
#ifndef ICM_20948_USE_DMP
#error "ICM_20948_USE_DMP NOT DEFINED! Needed for sensor fusion. Set it in platformio.ini or as a compile flag -e.g. CFLASG+=-DICM_20948_USE_DMP"
#endif


extern TaskHandle_t imuTaskHandle;
extern PubICM20948* imuPublisher;

// IMU task function to handle sensor initialization and data publishing.
void imuTask(void* pvParameters);

#endif  // IMU_ENABLED

#endif  // IMU_H
