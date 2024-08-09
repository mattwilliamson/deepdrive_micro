#ifndef PUB_ICM20948_H
#define PUB_ICM20948_H

#include <Arduino.h>
#include <EEPROM.h>
#include <MicroROS.h>
#include <Wire.h>
#include <sensor_msgs/msg/imu.h>

#include "ICM_20948.h"
#include "config.h"

/**
 * @class PubICM20948
 * @brief A class that initializes the ICM-20948 sensor and publishes IMU data as a ROS2 Imu message.
 */
class PubICM20948 {
 public:
  /**
   * @brief Constructor for the PubICM20948 class.
   *
   * @param frame_id The frame ID for the Imu message.
   * @param imu_topic_name The name of the ROS2 topic to publish the Imu data.
   * @param sensor A reference to an ICM_20948_I2C object to retrieve sensor data.
   * @param publish_rate The rate at which to publish the data in Hz.
   * @param support A pointer to the rclc_support_t structure for ROS2 support.
   * @param node A pointer to the rcl_node_t structure for ROS2 node.
   * @param executor A pointer to the rclc_executor_t structure for ROS2 executor.
   */
  PubICM20948(
      const char* frame_id,
      const char* imu_topic_name,
      ICM_20948_I2C& sensor,
      float publish_rate,
      rclc_support_t* support,
      rcl_node_t* node,
      rclc_executor_t* executor);

  /**
   * @brief Initializes the sensor and the ROS2 publishers.
   */
  void init();

  /**
   * @brief Updates the sensor data and publishes it if the publish rate interval has passed.
   */
  void update();

  /**
   * @brief Check all the available data and updates internal state.
   */
  void updateData();

  /**
   * @brief Get the publish rate.
   *
   * @return The publish rate in Hz.
   */
  float getPublishRate() const;

    /**
   * @brief Save the sensor biases to EEPROM after a couple minutes
   *
   * @return The publish rate in Hz.
   */
  void calibrate();

 private:
  const char* frame_id;
  rcl_publisher_t imu_publisher;
  sensor_msgs__msg__Imu* imu_msg;
  const char* imu_topic_name;
  ICM_20948_I2C& sensor;
  float publish_rate;
  unsigned long last_publish_time;
  icm_20948_DMP_data_t data;
  rclc_support_t* support;
  rcl_node_t* node;
  rclc_executor_t* executor;

  /**
   * @brief Helper function to convert quaternion to roll, pitch, and yaw.
   */
  void computeRPY();

  /**
   * @brief Struct for storing sensor biases.
   */
  struct BiasStore {
    int32_t header = 0x42;
    int32_t biasGyroX = 389952;
    int32_t biasGyroY = 23072;
    int32_t biasGyroZ = -133760;
    int32_t biasAccelX = 0;
    int32_t biasAccelY = 0;
    int32_t biasAccelZ = 0;
    int32_t biasCPassX = 1908864;
    int32_t biasCPassY = 1459936;
    int32_t biasCPassZ = 4079744;
    int32_t sum = 7727874;

    /**
     * @brief Calculates the checksum for the struct.
     *
     * @return The calculated checksum.
     */
    int32_t calculateChecksum() const {
      return header + biasGyroX + biasGyroY + biasGyroZ +
             biasAccelX + biasAccelY + biasAccelZ +
             biasCPassX + biasCPassY + biasCPassZ;
    }
  };

  BiasStore biases;
  bool biasesLoaded = false;

  /**
   * @brief Loads biases from EEPROM.
   *
   * @return True if loading is successful, false otherwise.
   */
  bool loadBiasesFromEEPROM();

  /**
   * @brief Saves biases to EEPROM.
   */
  void saveBiasesToEEPROM();

    /**
     * @brief Applies the stored biases to the sensor.
     */
    void applyBiasesToSensor();
};

#endif  // PUB_ICM20948_H
