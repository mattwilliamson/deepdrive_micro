#include "imu.h"

IMU::IMU(i2c_inst_t i2c, uint8_t address, uint8_t addressMag, int sda, int scl, int speed)
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

    float accel_g_[3] = {0.0f, 0.0f, 0.0f};
    float gyro_dps_[3] = {0.0f, 0.0f, 0.0f};
    float mag_ut_[3] = {0.0f, 0.0f, 0.0f};
    float temp_c_ = 0.0f;
    // TODO: Might use this flag to read in background task somehow
    has_new_data_ = false;

    filter_ = {0.5f, {1.0f, 0.0f, 0.0f, 0.0f}};

    // TODO: Store mag bias in flash or eeprom
    data_.mag_bias[0] = 102; data_.mag_bias[1] = -77; data_.mag_bias[2] = -227;
}


int8_t IMU::start() {
    // stdio_init_all();
    i2c_init(&i2c_, speed_);
    gpio_set_function(i2c_sda_, GPIO_FUNC_I2C);
    gpio_set_function(i2c_scl_, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_sda_);
    gpio_pull_up(i2c_scl_);

    return icm20948_init(&config_);
}

int8_t IMU::calibrate() {
    icm20948_cal_gyro(&config_, data_.gyro_bias);
    icm20948_cal_accel(&config_, data_.accel_bias);

    // The magnetometer calibration is a bit more involved, as you need to rotate the sensor around all axes
    // for some period of time. We will take a manual precalculated one for now.
    // icm20948_cal_mag_simple(&config_, data_.mag_bias);

    return 0;
}

void IMU::quaternianToEuler(madgwick_ahrs_t *data, float euler[]) {
    // 0: roll, 1: pitch, 2: yaw
    euler[0] = -1.0f * asinf(2.0f * (data->q[1]) * (data->q[3]) + 2.0f * (data->q[0]) * (data->q[2]));
    euler[1] = atan2f(2.0f * (data->q[2]) * (data->q[3]) - 2.0f * (data->q[0]) * (data->q[1]), 2.0f * (data->q[0]) * (data->q[0]) + 2.0f * (data->q[3]) * (data->q[3]) - 1.0f);
    euler[2] = atan2f(2.0f * (data->q[1]) * (data->q[2]) - 2.0f * (data->q[0]) * (data->q[3]), 2.0f * (data->q[0]) * (data->q[0]) + 2.0f * (data->q[1]) * (data->q[1]) - 1.0f);
    return;
}

int IMU::read() {
    icm20948_read_raw_accel(&config_, data_.accel_raw);
    icm20948_read_raw_gyro(&config_, data_.gyro_raw);
    icm20948_read_raw_mag(&config_, data_.mag_raw);
    icm20948_read_temp_c(&config_, &data_.temp_c);

    icm20948_read_cal_accel(&config_, &data_.accel_raw[0], &data_.accel_bias[0]);
    icm20948_read_cal_gyro(&config_, &data_.gyro_raw[0], &data_.gyro_bias[0]);
    icm20948_read_cal_mag(&config_, &data_.mag_raw[0], &data_.mag_bias[0]);
    icm20948_read_temp_c(&config_, &data_.temp_c);

    // accel(g)   = raw_value / (65535 / full_scale)
    // ex) if full_scale == +-4g then accel = raw_value / (65535 / 8) = raw_value / 8192
    // gyro(dps)  = raw_value / (65535 / full_scale)
    // ex) if full_scale == +-250dps then gyro = raw_value / (65535 / 500) = raw_value / 131
    // mag(uT)    = raw_value / (32752 / 4912) = (approx) (raw_value / 20) * 3
    // temp  = ((raw_value - ambient_temp) / speed_of_sound) + 21

    // 0: x, 1: y, 2: z
    accel_g_[0] = (float)data_.accel_raw[0] / 16384.0f * GRAVITY;
    accel_g_[1] = (float)data_.accel_raw[1] / 16384.0f * GRAVITY;
    accel_g_[2] = (float)data_.accel_raw[2] / 16384.0f * GRAVITY;
    gyro_dps_[0] = (float)data_.gyro_raw[0] / 131.0f;
    gyro_dps_[1] = (float)data_.gyro_raw[1] / 131.0f;
    gyro_dps_[2] = (float)data_.gyro_raw[2] / 131.0f;
    mag_ut_[0] = (float)data_.mag_raw[1];
    mag_ut_[1] = (float)-data_.mag_raw[0];
    mag_ut_[2] = (float)-data_.mag_raw[2];

    // Get Orientation by filtering raw data
    MadgwickAHRSupdate(&filter_, gyro_dps_[0], gyro_dps_[1], gyro_dps_[2], accel_g_[0], accel_g_[1], accel_g_[2], mag_ut_[0], mag_ut_[1], mag_ut_[2]);

    //MadgwickAHRSupdateIMU(&filter_, gyro_dps_[0], gyro_dps_[1], gyro_dps_[2], accel_g_[0] * 9.8, accel_g_[1] * 9.8, accel_g_[2] * 9.8);

    // float euler[3];
    // quaternianToEuler(&filter_, euler); // rad

    //printf("(data->q[0]) %0.1f, (data->q[1]) %0.1f, (data->q[2]) %0.1f, (data->q[3]) %0.1f\n", (data->q[0]), (data->q[1]), (data->q[2]), (data->q[3]));
    //printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler[0] * 57.29578f, euler[1] * 57.29578f, euler[2] * 57.29578f + 180.0f);
    // printf("D %0.1f %0.1f %0.1f\n", euler[2] * 57.29578f, euler[1] * 57.29578f, euler[0] * 57.29578f);

    // TODO: Calibration

    has_new_data_ = true;

    // TODO: error handling
    return 0;
}

Vector3 IMU::getAccel() { 
    return {
        accel_g_[0], 
        accel_g_[1], 
        accel_g_[2]
    }; 
}

Vector3 IMU::getGyro() {
    return {
        gyro_dps_[0] / RAD_TO_DEG, 
        gyro_dps_[1] / RAD_TO_DEG, 
        gyro_dps_[2] / RAD_TO_DEG}; 
}

Vector3 IMU::getMag() { 
    return {
        mag_ut_[0], 
        mag_ut_[1], 
        mag_ut_[2]
    }; 
}

Quaternion IMU::getOrientation() { 
    // w, x, y, z
    return Quaternion{filter_.q[0], filter_.q[1], filter_.q[2], filter_.q[3]}; 
}
