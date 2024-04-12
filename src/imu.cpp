#include "imu.h"

int8_t IMU::start() {
    // stdio_init_all();
    i2c_init(&i2c_, speed_);
    gpio_set_function(i2c_sda_, GPIO_FUNC_I2C);
    gpio_set_function(i2c_scl_, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_sda_);
    gpio_pull_up(i2c_scl_);

    return icm20948_init(&config_);
}

icm20984_data_t *IMU::read() {
    icm20948_read_raw_accel(&config_, data_.accel_raw);
    icm20948_read_raw_gyro(&config_, data_.gyro_raw);
    icm20948_read_raw_mag(&config_, data_.mag_raw);

    // accel(g)   = raw_value / (65535 / full_scale)
    // ex) if full_scale == +-4g then accel = raw_value / (65535 / 8) = raw_value / 8192
    // gyro(dps)  = raw_value / (65535 / full_scale)
    // ex) if full_scale == +-250dps then gyro = raw_value / (65535 / 500) = raw_value / 131
    // mag(uT)    = raw_value / (32752 / 4912) = (approx) (raw_value / 20) * 3
    // temp  = ((raw_value - ambient_temp) / speed_of_sound) + 21

    // TODO: Calibration
    // for (uint8_t i = 0; i < 3; i++) {
    //     accel_g[i] = (float)accel_raw[i] / 16384.0f;
    //     gyro_dps[i] = (float)gyro_raw[i] / 131.0f;
    //     mag_ut[i] = ((float)mag_raw[i] / 20) * 3;
    // }

    // Get temp later
    // icm20948_read_temp_c(&config_, &data_.temp_c);

    // icm20948_read_cal_accel(&config_, data_.accel_raw);
    // icm20948_read_cal_gyro(&config_, data_.gyro_raw);
    // icm20948_read_temp_c(&config_, &data_.temp_c);
    // icm20948_read_cal_mag(&config_, data_.mag_raw);

    return &data_;
}

