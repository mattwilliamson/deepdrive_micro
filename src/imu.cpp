#include "imu.h"

#ifdef IMU_ENABLED

// Global variables to hold the task handle and the publisher object
PubICM20948* imuPublisher = NULL;
ICM_20948_I2C icm20948;

/**
 * @brief IMU task function to handle sensor initialization and data publishing.
 * 
 * @param pvParameters A pointer to the task parameters (not used in this task).
 */
void imuTask(void* pvParameters) {
    SerialDebug.println("starting imuTask");
    imuPublisher = new PubICM20948(IMU_FRAME_ID, IMU_TOPIC_NAME, icm20948, IMU_PUBLISH_RATE, &support, &node, &executor);
    SerialDebug.println("Initializing IMU...");
    imuPublisher->init();
    SerialDebug.println("imu initialized");
    delay(1000);

    while (true) {
        imuPublisher->update();
        // vTaskDelay(pdMS_TO_TICKS(1));
        delay(1);
    }
}

#endif  // IMU_ENABLED
