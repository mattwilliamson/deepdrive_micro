#include "imu.hpp"

IMU::IMU(i2c_inst_t i2c, uint8_t address, uint8_t addressMag, int sda, int scl,
         int speed) {
  i2c_ = i2c;
  address_ = address;
  addressMag_ = addressMag;
  i2c_sda_ = sda;
  i2c_scl_ = scl;
  speed_ = speed;
  imu_ = ICM_20948_I2C();

  // config_ = {
  //     .addr_accel_gyro = address_, .addr_mag = addressMag_, .i2c = &i2c_};

  data_ = {.accel_raw = {0, 0, 0},
           .accel_bias = {0, 0, 0},
           .gyro_raw = {0},
           .gyro_bias = {0},
           .mag_raw = {0},
           .mag_bias = {0},
           .temp_c = 0.0f};

  float accel_g_[3] = {0.0f, 0.0f, 0.0f};
  float gyro_dps_[3] = {0.0f, 0.0f, 0.0f};
  float mag_ut_[3] = {0.0f, 0.0f, 0.0f};
  float temp_c_ = 0.0f;
  // // TODO: Might use this flag to read in background task somehow
  has_new_data_ = false;

  // filter_ = {0.5f, {1.0f, 0.0f, 0.0f, 0.0f}};

  // // TODO: Store mag bias in flash or eeprom
  data_.mag_bias[0] = 102;
  data_.mag_bias[1] = -77;
  data_.mag_bias[2] = -227;

  error = ImuErrorCode::INIT;
}

int8_t IMU::start() {
  // stdio_init_all();
  i2c_init(&i2c_, speed_);
  gpio_set_function(i2c_sda_, GPIO_FUNC_I2C);
  gpio_set_function(i2c_scl_, GPIO_FUNC_I2C);
  gpio_pull_up(i2c_sda_);
  gpio_pull_up(i2c_scl_);

  imu_.begin(&i2c_, address_);

  // Enable DMP
  if (imu_.initializeDMP() != ICM_20948_Stat_Ok) {
    return ImuErrorCode::DMP_FAILED;
  }

  // Raw Accelerometer
  // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);

  // 9 axis quaternion
  if (imu_.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) != ICM_20948_Stat_Ok) {
    return ImuErrorCode::DMP_FAILED;
  }

  // Max speed 5hz?
  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  if (imu_.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) != ICM_20948_Stat_Ok) {
    return ImuErrorCode::DMP_FAILED;
  }

  // Enable FIFO queue
  if (imu_.enableFIFO() != ICM_20948_Stat_Ok) {
    return ImuErrorCode::DMP_FAILED;
  }

  // Enable the DMP
  if (imu_.enableDMP() != ICM_20948_Stat_Ok) {
    return ImuErrorCode::DMP_FAILED;
  }

  // Reset DMP
  if (imu_.resetDMP() != ICM_20948_Stat_Ok) {
    error = ImuErrorCode::DMP_FAILED;
  }

  // Reset FIFO
  if (imu_.resetFIFO() != ICM_20948_Stat_Ok) {
    error = ImuErrorCode::DMP_FAILED;
  }

  // TODO: Show status in dialod
  // SERIAL_PORT.println(myICM.statusString());
  //   if (myICM.status != ICM_20948_Stat_Ok)

  // return icm20948_init(&config_);
  return 0;
}

int8_t IMU::calibrate() {
  // TODO: Calibrate accelerometer and gyro by reading a bunch of times and averaging out to get the bias
  // icm20948_cal_gyro(&config_, data_.gyro_bias);
  // icm20948_cal_accel(&config_, data_.accel_bias);

  // The magnetometer calibration is a bit more involved, as you need to rotate
  // the sensor around all axes for some period of time. We will take a manual
  // precalculated one for now. icm20948_cal_mag_simple(&config_,
  // data_.mag_bias);

  imu_.setBiasCPassX(data_.mag_bias[0]);
  imu_.setBiasCPassY(data_.mag_bias[1]);
  imu_.setBiasCPassZ(data_.mag_bias[2]);

  return 0;
}

std::array<float, 3> IMU::quaternianToEuler(const Quaternion& quaternion) {
  std::array<float, 3> euler;
  euler[0] = -1.0f * asinf(2.0f * (quaternion.x) * (quaternion.z) +
                           2.0f * (quaternion.w) * (quaternion.y));
  euler[1] = atan2f(
      2.0f * (quaternion.y) * (quaternion.z) - 2.0f * (quaternion.w) * (quaternion.x),
      2.0f * (quaternion.w) * (quaternion.w) + 2.0f * (quaternion.z) * (quaternion.z) -
          1.0f);
  euler[2] = atan2f(
      2.0f * (quaternion.x) * (quaternion.y) - 2.0f * (quaternion.w) * (quaternion.z),
      2.0f * (quaternion.w) * (quaternion.w) + 2.0f * (quaternion.x) * (quaternion.x) -
          1.0f);
  return euler;
}

ImuErrorCode IMU::read() {
  // imu_.getAGMT();
  // data_.accel_raw[0] = imu_.accX();
  // data_.accel_raw[1] = imu_.accY();
  // data_.accel_raw[2] = imu_.accZ();
  // data_.gyro_raw[0] = imu_.gyrX();
  // data_.gyro_raw[1] = imu_.gyrY();
  // data_.gyro_raw[2] = imu_.gyrZ();
  // data_.mag_raw[0] = imu_.magX();
  // data_.mag_raw[1] = imu_.magY();
  // data_.mag_raw[2] = imu_.magZ();
  // data_.temp_c = imu_.temp();

  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  imu_.readDMPdataFromFIFO(&dmp_data_);

  if ((imu_.status != ICM_20948_Stat_Ok) && (imu_.status != ICM_20948_Stat_FIFOMoreDataAvail)) {
    return ImuErrorCode::NO_DATA;
  }

  if ((dmp_data_.header & DMP_header_bitmap_Quat9) > 0) {  // We have asked for orientation data so we should receive Quat9
    // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
    // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
    // The quaternion data is scaled by 2^30.

    // SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);
    //  TODO: Handle accuracy

    // Scale to +/- 1
    const double scale = pow(2, 30);  // 1073741824.0 2^30

    double q1 = dmp_data_.Quat9.Data.Q1;
    double q2 = dmp_data_.Quat9.Data.Q2;
    double q3 = dmp_data_.Quat9.Data.Q3;

    q1 /= scale;  // 0.31
    q2 /= scale;  // -1.94
    q3 /= scale;  // 1.01

    // // double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
    // double q0 = ((q1 * q1) + (q2 * q2) + (q3 * q3));  // 4.9
    // q0 = 1.0 - q0;                                    // -3.9
    // q0 = sqrt(q0);                                    // -inf
    // if (q0 < 0) {
    //   q0 = 1.0;
    // }

    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
    // double q0 = dmp_data_.Quat9.Data.Q0 / 1073741824.0; // Convert to double. Divide by 2^30

    orientation_ = Quaternion{q0, q1, q2, q3};

    // orientation_.normalize();

    has_new_data_ = true;

    // TODO: error handling
    return ImuErrorCode::OK;
  }

  // else return error

  // Old library:
  // icm20948_read_raw_accel(&config_, data_.accel_raw);
  // icm20948_read_raw_gyro(&config_, data_.gyro_raw);
  // icm20948_read_raw_mag(&config_, data_.mag_raw);
  // icm20948_read_temp_c(&config_, &data_.temp_c);

  // icm20948_read_cal_accel(&config_, &data_.accel_raw[0], &data_.accel_bias[0]);
  // icm20948_read_cal_gyro(&config_, &data_.gyro_raw[0], &data_.gyro_bias[0]);
  // icm20948_read_cal_mag(&config_, &data_.mag_raw[0], &data_.mag_bias[0]);
  // icm20948_read_temp_c(&config_, &data_.temp_c);

  // accel(g)   = raw_value / (65535 / full_scale)
  // ex) if full_scale == +-4g then accel = raw_value / (65535 / 8) = raw_value
  // / 8192 gyro(dps)  = raw_value / (65535 / full_scale) ex) if full_scale ==
  // +-250dps then gyro = raw_value / (65535 / 500) = raw_value / 131 mag(uT) =
  // raw_value / (32752 / 4912) = (approx) (raw_value / 20) * 3 temp  =
  // ((raw_value - ambient_temp) / speed_of_sound) + 21

  // 0: x, 1: y, 2: z
  // accel_g_[0] = (float)data_.accel_raw[0] / 16384.0f * GRAVITY;
  // accel_g_[1] = (float)data_.accel_raw[1] / 16384.0f * GRAVITY;
  // accel_g_[2] = (float)data_.accel_raw[2] / 16384.0f * GRAVITY;
  // gyro_dps_[0] = (float)data_.gyro_raw[0] / 131.0f;
  // gyro_dps_[1] = (float)data_.gyro_raw[1] / 131.0f;
  // gyro_dps_[2] = (float)data_.gyro_raw[2] / 131.0f;
  // mag_ut_[0] = (float)data_.mag_raw[1];
  // mag_ut_[1] = (float)-data_.mag_raw[0];
  // mag_ut_[2] = (float)-data_.mag_raw[2];

  // accel_g_[0] = (float)data_.accel_raw[0] / GRAVITY;
  // accel_g_[1] = (float)data_.accel_raw[1] / GRAVITY;
  // accel_g_[2] = (float)data_.accel_raw[2] / GRAVITY;

  // Get Orientation by filtering raw data
  // MadgwickAHRSupdate(&filter_, gyro_dps_[0], gyro_dps_[1], gyro_dps_[2],
  //                    accel_g_[0], accel_g_[1], accel_g_[2], mag_ut_[0],
  //                    mag_ut_[1], mag_ut_[2]);

  // MadgwickAHRSupdateIMU(&filter_, gyro_dps_[0], gyro_dps_[1], gyro_dps_[2],
  // accel_g_[0] * 9.8, accel_g_[1] * 9.8, accel_g_[2] * 9.8);

  // float euler[3];
  // quaternianToEuler(&filter_, euler); // rad

  // printf("(data->q[0]) %0.1f, (data->q[1]) %0.1f, (data->q[2]) %0.1f,
  // (data->q[3]) %0.1f\n", (data->q[0]), (data->q[1]), (data->q[2]),
  // (data->q[3])); printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler[0]
  // * 57.29578f, euler[1] * 57.29578f, euler[2] * 57.29578f + 180.0f);
  //  printf("D %0.1f %0.1f %0.1f\n", euler[2] * 57.29578f, euler[1]
  //  * 57.29578f, euler[0] * 57.29578f);

  // TODO: Calibration

  return ImuErrorCode::ERROR;
}

Vector3 IMU::getAccel() {
  return {accel_g_[0] * GRAVITY, accel_g_[1] * GRAVITY, accel_g_[2] * GRAVITY};
}

Vector3 IMU::getGyro() {
  return {gyro_dps_[0] / RAD_TO_DEG, gyro_dps_[1] / RAD_TO_DEG, gyro_dps_[2] / RAD_TO_DEG};
}

Vector3 IMU::getMag() {
  return {mag_ut_[0], mag_ut_[1], mag_ut_[2]};
}

Quaternion IMU::getOrientation() {
  // w, x, y, z
  // return Quaternion{filter_.q[0], filter_.q[1], filter_.q[2], filter_.q[3]};
  return orientation_;
}
