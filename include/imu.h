#pragma once

#include "config.h"

extern "C" {
    #include "hardware/i2c.h"
    #include "pico-icm20948.h"
}
class IMU {
public:
    // i2c = {i2c0_hw, false}
    IMU(i2c_inst_t i2c = IMU_I2C, 
        uint8_t address = IMU_ADDRESS, 
        uint8_t addressMag = IMU_ADDRESS_MAG, 
        int sda = IMU_I2C_SDA, 
        int scl = IMU_I2C_SCL,
        int speed = IMU_I2C_SPEED) 
    {
        i2c_ = i2c;
        address_ = address;
        addressMag_ = addressMag;
        i2c_sda_ = sda;
        i2c_scl_ = scl;
        speed_ = speed;

        config_ = {
            .addr_accel_gyro = address_,
            .addr_mag = addressMag_,
            .i2c = &i2c_
        };

        data_ = {
            .accel_raw = {0, 0, 0},
            .accel_bias = {0, 0, 0},
            .gyro_raw = {0},
            .gyro_bias = {0},
            .mag_raw = {0},
            .mag_bias = {0},
            .temp_c = 0.0f
        };
    }

    ~IMU() {}
    int8_t start();
    icm20984_data_t *read();

private:
    i2c_inst_t i2c_;
    uint8_t address_;
    uint8_t addressMag_;
    int i2c_sda_;
    int i2c_scl_;
    int speed_;
    
        
    icm20948_config_t config_;
    icm20984_data_t data_;
};