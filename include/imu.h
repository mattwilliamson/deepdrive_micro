#pragma once

#include "config.h"

// https://invensense.tdk.com/download-pdf/icm-20948-datasheet/

// # This is a message to hold data from an IMU (Inertial Measurement Unit)
// #
// # Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
// #
// # If the covariance of the measurement is known, it should be filled in (if all you know is the 
// # variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
// # A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
// # data a covariance will have to be assumed or gotten from some other source
// #
// # If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
// # estimate), please set element 0 of the associated covariance matrix to -1
// # If you are interpreting this message, please check for a value of -1 in the first element of each 
// # covariance matrix, and disregard the associated estimate.

extern "C" {
    #include "hardware/i2c.h"
    #include "pico-icm20948.h"
    #include "MadgwickAHRS.h"
}

#include <cmath>

struct Vector3 {
    float x;
    float y;
    float z;
};

struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

const float RAD_TO_DEG = 57.29578;
const float GRAVITY = 9.81;

/**
 * @brief Represents an Inertial Measurement Unit (IMU) sensor.
 * 
 * The IMU class provides methods to interact with an IMU sensor, including reading sensor data,
 * calibrating the sensor, and retrieving specific sensor measurements such as acceleration,
 * gyroscope, magnetometer, and orientation.
 */
class IMU {
public:
    /**
     * @brief Constructs an IMU object with the specified parameters.
     * 
     * @param i2c The I2C instance to be used for communication with the IMU sensor.
     * @param address The I2C address of the IMU sensor.
     * @param addressMag The I2C address of the magnetometer within the IMU sensor.
     * @param sda The SDA pin number for I2C communication.
     * @param scl The SCL pin number for I2C communication.
     * @param speed The I2C bus speed in Hz.
     */
    IMU(i2c_inst_t i2c = IMU_I2C, 
        uint8_t address = IMU_ADDRESS, 
        uint8_t addressMag = IMU_ADDRESS_MAG, 
        int sda = IMU_I2C_SDA, 
        int scl = IMU_I2C_SCL,
        int speed = IMU_I2C_SPEED);
        
    /**
     * @brief Destroys the IMU object.
     */
    ~IMU() {}

    /**
     * @brief Starts the IMU sensor.
     * 
     * @return An integer value indicating the success or failure of starting the sensor.
     */
    int8_t start();

    /**
     * @brief Calibrates the IMU sensor.
     * 
     * @return An integer value indicating the success or failure of the calibration process.
     */
    int8_t calibrate();

    /**
     * @brief Reads sensor data from the IMU sensor.
     * 
     * @return An integer value indicating the success or failure of reading the sensor data.
     */
    int read();

    /**
     * @brief Converts the quaternion representation of orientation to Euler angles.
     * 
     * @param data Pointer to the data structure containing the quaternion representation of orientation.
     * @param euler Array to store the resulting Euler angles.
     */
    static void quaternianToEuler(madgwick_ahrs_t *data, float euler[]);

    /**
     * @brief Retrieves the acceleration measurements from the IMU sensor.
     * 
     * @return A Vector3 object representing the acceleration in three dimensions in m/s^2.
     */
    Vector3 getAccel();

    /**
     * @brief Retrieves the gyroscope measurements from the IMU sensor.
     * 
     * @return A Vector3 object representing the gyroscope readings in three dimensions in rad/s.
     */
    Vector3 getGyro();

    /**
     * @brief Retrieves the magnetometer measurements from the IMU sensor.
     * 
     * @return A Vector3 object representing the magnetometer readings in three dimensions in uT.
     */
    Vector3 getMag();

    /**
     * @brief Retrieves the orientation of the IMU sensor.
     * 
     * @return A Quaternion object representing the estimated orientation of the sensor.
     */
    Quaternion getOrientation();

private:
    i2c_inst_t i2c_; // The I2C instance used for communication with the IMU sensor.
    uint8_t address_; // The I2C address of the IMU sensor.
    uint8_t addressMag_; // The I2C address of the magnetometer within the IMU sensor.
    int i2c_sda_; // The SDA pin number for I2C communication.
    int i2c_scl_; // The SCL pin number for I2C communication.
    int speed_; // The I2C bus speed in Hz.
    
    icm20948_config_t config_; // Configuration settings for the IMU sensor.
    icm20984_data_t data_; // Data structure to store sensor readings.
    madgwick_ahrs_t filter_; // Data structure for sensor fusion algorithm.

    float accel_g_[3]; // Acceleration measurements in g-force.
    float gyro_dps_[3]; // Gyroscope readings in degrees per second.
    float mag_ut_[3]; // Magnetometer readings in microteslas.
    float temp_c_; // Temperature reading in degrees Celsius.

    bool has_new_data_; // Flag indicating whether new sensor data is available.
};
