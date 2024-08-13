#include "PubICM20948.h"

/**
 * @brief Constructs a PubICM20948 object.
 *
 * Initializes the class with the specified ROS2 topics, sensor, and rates for publishing.
 *
 * @param frame_id The frame ID for the Imu message.
 * @param imu_topic_name The name of the ROS2 topic to publish the Imu data.
 * @param sensor A reference to an ICM_20948_I2C object to retrieve sensor data.
 * @param publish_rate The rate at which to publish the data in Hz.
 * @param support A pointer to the rclc_support_t structure for ROS2 support.
 * @param node A pointer to the rcl_node_t structure for ROS2 node.
 * @param executor A pointer to the rclc_executor_t structure for ROS2 executor.
 */
PubICM20948::PubICM20948(
    const char* frame_id,
    const char* imu_topic_name,
    ICM_20948_I2C& sensor,
    float publish_rate,
    rclc_support_t* support,
    rcl_node_t* node,
    rclc_executor_t* executor)
    : frame_id(frame_id),
      imu_topic_name(imu_topic_name),
      sensor(sensor),
      publish_rate(publish_rate),
      last_publish_time(0),
      support(support),
      node(node),
      executor(executor) {
  // Initialize the Imu message
  imu_msg = sensor_msgs__msg__Imu__create();
  imu_msg->header.frame_id = micro_ros_string_utilities_init(frame_id);

  // Initialize orientation to 0
  imu_msg->orientation.x = 0.0;
  imu_msg->orientation.y = 0.0;
  imu_msg->orientation.z = 0.0;
  imu_msg->orientation.w = 1.0;

  // Initialize covariance values
  for (size_t i = 0; i < 9; ++i) {
    imu_msg->orientation_covariance[i] = IMU_MIN_COVARIANCE;
    imu_msg->angular_velocity_covariance[i] = IMU_MIN_COVARIANCE;
    imu_msg->linear_acceleration_covariance[i] = IMU_MIN_COVARIANCE;
  }
}

/**
 * @brief Flags the sensor to calibrate and save to eeprom.
 * Must be called after init.
 */
void PubICM20948::calibrate() {
  biasesLoaded = false;
}

/**
 * @brief Initializes the sensor and the ROS2 publishers.
 */
void PubICM20948::init() {
  SerialDebug.println("IMU initialize I2C");
  // Initialize the sensor
  IMU_WIRE_PORT.begin();
  IMU_WIRE_PORT.setClock(IMU_I2C_SPEED);
  SerialDebug.println("IMU sensor.begin");
  sensor.begin(IMU_WIRE_PORT, IMU_AD0_VAL);

  int retry_count = 0;
  while (sensor.status != ICM_20948_Stat_Ok && retry_count < IMU_MAX_RETRY_COUNT) {
    SerialDebug.println("IMU initialization failed! Retrying..." + String(retry_count + 1) + "/" + String(IMU_MAX_RETRY_COUNT));
    SerialDebug.println(sensor.statusString());
    delay(IMU_RETRY_DELAY_MS);
    sensor.begin(IMU_WIRE_PORT, IMU_AD0_VAL);
    retry_count++;
  }

  if (sensor.status != ICM_20948_Stat_Ok) {
    SerialDebug.println("IMU initialization failed after " + String(IMU_MAX_RETRY_COUNT) + " retries!");
    SerialDebug.println(sensor.statusString());
    return;
  }

  // Load biases from EEPROM if available
  if (loadBiasesFromEEPROM()) {
    SerialDebug.println("IMU load biases");
    // Apply the biases to the sensor
    sensor.setBiasGyroX(biases.biasGyroX);
    sensor.setBiasGyroY(biases.biasGyroY);
    sensor.setBiasGyroZ(biases.biasGyroZ);
    sensor.setBiasAccelX(biases.biasAccelX);
    sensor.setBiasAccelY(biases.biasAccelY);
    sensor.setBiasAccelZ(biases.biasAccelZ);
  }

  // Enable DMP (Digital Motion Processor)
  bool success;

  for (int i = 0; i < IMU_MAX_RETRY_COUNT; i++) {
    SerialDebug.println("IMU enabling dmp");
    success = true;
    success &= (sensor.initializeDMP() == ICM_20948_Stat_Ok);

    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    // Enable the DMP orientation sensor
    success &= (sensor.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    if (!success) {
      SerialDebug.println("Enable DMP Orientation failed!");
    }

    // Enable any additional sensors / features
    success &= (sensor.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);
    if (!success) {
      SerialDebug.println("Enable DMP Gyroscope failed!");
    }
    success &= (sensor.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
    if (!success) {
      SerialDebug.println("Enable DMP Accelerometer failed!");
    }
    success &= (sensor.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);
    if (!success) {
      SerialDebug.println("Enable DMP Magnetometer failed!");
    }

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    success &= (sensor.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok);  // Set to the maximum
    success &= (sensor.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok);  // Set to the maximum
    // success &= (sensor.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (sensor.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok);  // Set to the maximum
    // success &= (sensor.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (sensor.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok);  // Set to the maximum

    // Enable the FIFO
    success &= (sensor.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (sensor.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (sensor.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (sensor.resetFIFO() == ICM_20948_Stat_Ok);

    if (!success) {
      SerialDebug.println(F("Enable DMP failed!"));
      delay(IMU_RETRY_DELAY_MS);
    } else {
      break;
    }
  }

  if (!success) {
    SerialDebug.println(F("Enable DMP failed after retrying!"));
    return;
  }

  SerialDebug.println(F("IMU initialized with DMP successfully!"));

  // Initialize the Imu publisher
  rclc_publisher_init_default(
      &imu_publisher,
      node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      imu_topic_name);
}

/**
 * @brief Check all the available data and updates internal state.
 */
void PubICM20948::updateData() {
  if ((sensor.status == ICM_20948_Stat_Ok) || (sensor.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data.header & DMP_header_bitmap_Quat9) > 0) {
      // Scale to +/- 1
      double x = ((double)data.Quat9.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
      double y = ((double)data.Quat9.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
      double z = ((double)data.Quat9.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30
      // double x = data.Quat9.Data.Q1;  // Convert to double. Divide by 2^30
      // double y = data.Quat9.Data.Q2;  // Convert to double. Divide by 2^30
      // double z = data.Quat9.Data.Q3;  // Convert to double. Divide by 2^30
      // double w = 1073741824.0;

      // // Calculate w component
      double wSquared = 1.0 - ((double)(x * x) + (y * y) + (z * z));
      double w = (wSquared > 0.0) ? sqrtf(wSquared) : 0.0;

      // // Recalculate the magnitude after calculating w
      double magnitude = sqrt((double)(x * x) + (y * y) + (z * z) + (w * w));

      // Normalize the quaternion components
      if (magnitude > 0.0) {
        x /= magnitude;
        y /= magnitude;
        z /= magnitude;
        w /= magnitude;
      }

      imu_msg->orientation.x = x;
      imu_msg->orientation.y = y;
      imu_msg->orientation.z = z;
      imu_msg->orientation.w = w;

      // This is just for debugging purposes
      magnitude = sqrt((double)(x * x) + (y * y) + (z * z) + (w * w));

      // TODO: Publish compass
    }
  }
}

/**
 * @brief Updates the sensor data and publishes it if the publish rate interval has passed.
 */
void PubICM20948::update() {
  unsigned long current_time = millis();

  // #ifdef IMU_ENABLE_EEPROM
  if (!biasesLoaded) {
    static unsigned long last_bias_save_time = 0;
    static unsigned long last_bias_check_time = 0;
    if (last_bias_save_time == 0) {
      SerialDebug.println("Calibrating IMU! Leave the IMU still for a few seconds then move it around in all directions. It will be saved after 2 minutes.");
      last_bias_save_time = current_time;
    } else if (current_time - last_bias_save_time >= IMU_BIAS_SAVE_INTERVAL_MS) {
      saveBiasesToEEPROM();
      last_bias_save_time = current_time;
    } else if (current_time - last_bias_check_time >= 1000) {
      SerialDebug.println("Calibrating IMU! Time remaining: " + String((IMU_BIAS_SAVE_INTERVAL_MS - (current_time - last_bias_save_time)) / 1000) + " seconds.");
      last_bias_check_time = current_time;
    }
  }
  // #endif

  // Read latest data from the FIFO
  sensor.readDMPdataFromFIFO(&data);
  updateData();

  while (sensor.status == ICM_20948_Stat_FIFOMoreDataAvail) {
    // Read all available data from the FIFO
    sensor.readDMPdataFromFIFO(&data);
    updateData();
  }

  // Publish data if publish rate interval has passed
  if (current_time - last_publish_time >= (1000.0 / publish_rate)) {
    if (sensor.dataReady()) {
      sensor.getAGMT();  // Get the latest data from the sensor

      // # Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
      // Populate the Imu message
      imu_msg->header.stamp.sec = current_time / 1000;
      imu_msg->header.stamp.nanosec = (current_time % 1000) * 1000000;

      // Conversion factors
      const float dps_to_rads = 3.14159265358979323846 / 180.0;
      const float g_to_mss = 9.80665;

      // Convert gyroscope readings from dps to rad/s
      imu_msg->angular_velocity.x = sensor.gyrX() * dps_to_rads;
      imu_msg->angular_velocity.y = sensor.gyrY() * dps_to_rads;
      imu_msg->angular_velocity.z = sensor.gyrZ() * dps_to_rads;

      // Convert accelerometer readings from mG to m/s^2
      imu_msg->linear_acceleration.x = sensor.accX() / 1000.0 * g_to_mss;
      imu_msg->linear_acceleration.y = sensor.accY() / 1000.0 * g_to_mss;
      imu_msg->linear_acceleration.z = sensor.accZ() / 1000.0 * g_to_mss;

      // Publish the Imu message
      RCSOFTCHECK(rcl_publish(&imu_publisher, imu_msg, NULL));

      last_publish_time = current_time;
    }
  }
}

/**
 * @brief Get the publish rate.
 *
 * @return The publish rate in Hz.
 */
float PubICM20948::getPublishRate() const {
  return publish_rate;
}

/**
 * @brief Loads biases from EEPROM.
 *
 * @return True if loading is successful, false otherwise.
 */
bool PubICM20948::loadBiasesFromEEPROM() {
#ifdef IMU_ENABLE_EEPROM

  // Allocate 256 Bytes for EEPROM storage. ESP32 needs this.
  EEPROM.begin(256);
  EEPROM.get(0, biases);

  // Validate header and checksum
  if (biases.header != 0x42) {
    SerialDebug.println("Biases EEPROM Header doesn't match.");
    return false;
  }

  SerialDebug.println("Biases values:");

  SerialDebug.print("Gyro X Bias: ");
  SerialDebug.println(biases.biasGyroX);

  SerialDebug.print("Gyro Y Bias: ");
  SerialDebug.println(biases.biasGyroY);

  SerialDebug.print("Gyro Z Bias: ");
  SerialDebug.println(biases.biasGyroZ);

  SerialDebug.print("Accel X Bias: ");
  SerialDebug.println(biases.biasAccelX);

  SerialDebug.print("Accel Y Bias: ");
  SerialDebug.println(biases.biasAccelY);

  SerialDebug.print("Accel Z Bias: ");
  SerialDebug.println(biases.biasAccelZ);

  SerialDebug.print("CPass X Bias: ");
  SerialDebug.println(biases.biasCPassX);

  SerialDebug.print("CPass Y Bias: ");
  SerialDebug.println(biases.biasCPassY);

  SerialDebug.print("CPass Z Bias: ");
  SerialDebug.println(biases.biasCPassZ);

  SerialDebug.print("Checksum: ");
  SerialDebug.println(biases.sum);

  if (biases.sum != biases.calculateChecksum()) {
    SerialDebug.println("Failed to load biases from EEPROM.");
    return false;
  }

  SerialDebug.println("Biases successfully loaded from EEPROM.");
  biasesLoaded = true;

  return false;

#else
  biasesLoaded = true;
  // TODO: publish bias values to diagnostic

  return true;
#endif
}

/**
 * @brief Saves biases to EEPROM.
 */
void PubICM20948::saveBiasesToEEPROM() {
  SerialDebug.println("Saving biases to EEPROM...");

  bool success = true;

  // Read biases from the sensor
  success &= (sensor.getBiasGyroX(&biases.biasGyroX) == ICM_20948_Stat_Ok);
  success &= (sensor.getBiasGyroY(&biases.biasGyroY) == ICM_20948_Stat_Ok);
  success &= (sensor.getBiasGyroZ(&biases.biasGyroZ) == ICM_20948_Stat_Ok);
  success &= (sensor.getBiasAccelX(&biases.biasAccelX) == ICM_20948_Stat_Ok);
  success &= (sensor.getBiasAccelY(&biases.biasAccelY) == ICM_20948_Stat_Ok);
  success &= (sensor.getBiasAccelZ(&biases.biasAccelZ) == ICM_20948_Stat_Ok);
  success &= (sensor.getBiasCPassX(&biases.biasCPassX) == ICM_20948_Stat_Ok);
  success &= (sensor.getBiasCPassY(&biases.biasCPassY) == ICM_20948_Stat_Ok);
  success &= (sensor.getBiasCPassZ(&biases.biasCPassZ) == ICM_20948_Stat_Ok);

  if (success) {
    // Update the checksum
    biases.sum = biases.calculateChecksum();

// Write to EEPROM
#ifdef IMU_ENABLE_EEPROM
    EEPROM.put(0, biases);
    EEPROM.commit();  // Ensure the data is written to EEPROM
#endif

    SerialDebug.println("Biases values:");

    SerialDebug.print("Gyro X Bias: ");
    SerialDebug.println(biases.biasGyroX);

    SerialDebug.print("Gyro Y Bias: ");
    SerialDebug.println(biases.biasGyroY);

    SerialDebug.print("Gyro Z Bias: ");
    SerialDebug.println(biases.biasGyroZ);

    SerialDebug.print("Accel X Bias: ");
    SerialDebug.println(biases.biasAccelX);

    SerialDebug.print("Accel Y Bias: ");
    SerialDebug.println(biases.biasAccelY);

    SerialDebug.print("Accel Z Bias: ");
    SerialDebug.println(biases.biasAccelZ);

    SerialDebug.print("CPass X Bias: ");
    SerialDebug.println(biases.biasCPassX);

    SerialDebug.print("CPass Y Bias: ");
    SerialDebug.println(biases.biasCPassY);

    SerialDebug.print("CPass Z Bias: ");
    SerialDebug.println(biases.biasCPassZ);

    SerialDebug.print("Checksum: ");
    SerialDebug.println(biases.sum);

    SerialDebug.println("Biases successfully saved to EEPROM.");

    biasesLoaded = true;
  } else {
    SerialDebug.println("Failed to read biases from the sensor.");
  }
}

/**
 * @brief Applies the stored biases to the sensor.
 */
void PubICM20948::applyBiasesToSensor() {
  sensor.setBiasGyroX(biases.biasGyroX);
  sensor.setBiasGyroY(biases.biasGyroY);
  sensor.setBiasGyroZ(biases.biasGyroZ);
  sensor.setBiasAccelX(biases.biasAccelX);
  sensor.setBiasAccelY(biases.biasAccelY);
  sensor.setBiasAccelZ(biases.biasAccelZ);
  sensor.setBiasCPassX(biases.biasCPassX);
  sensor.setBiasCPassY(biases.biasCPassY);
  sensor.setBiasCPassZ(biases.biasCPassZ);
}