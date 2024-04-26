#include "imu.hpp"

#include "node.hpp"

IMU::IMU(i2c_inst_t i2c, uint8_t address, uint8_t addressMag, int sda, int scl, int speed) {
  i2c_ = i2c;
  address_ = address;
  addressMag_ = addressMag;
  i2c_sda_ = sda;
  i2c_scl_ = scl;
  speed_ = speed;
  // imu_ = ICM_20948_I2C();

  config_ = {.addr_accel_gyro = address_, .addr_mag = addressMag_, .i2c = &i2c_};

  // data_ = {.accel_raw = {0, 0, 0},
  //          .accel_bias = {0, 0, 0},
  //          .gyro_raw = {0},
  //          .gyro_bias = {0},
  //          .mag_raw = {0},
  //          .mag_bias = {0},
  //          .temp_c = 0.0f};

  float accel_g_[3] = {0.0f, 0.0f, 0.0f};
  float gyro_dps_[3] = {0.0f, 0.0f, 0.0f};
  float mag_ut_[3] = {0.0f, 0.0f, 0.0f};
  float temp_c_ = 0.0f;
  // // TODO: Might use this flag to read in background task somehow
  has_new_data_ = false;

  // filter_ = {0.5f, {1.0f, 0.0f, 0.0f, 0.0f}};

  // TODO: Store mag bias in flash or eeprom
  // data_.mag_bias[0] = 102;
  // data_.mag_bias[1] = -77;
  // data_.mag_bias[2] = -227;

  FusionOffsetInitialise(&offset, IMU_LOOP_HZ);
  FusionAhrsInitialise(&ahrs);

  // Set AHRS algorithm settings
  const FusionAhrsSettings settings = {
      .convention = FusionConventionNwu,
      .gain = 0.5f,
      .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
      .accelerationRejection = 10.0f,
      .magneticRejection = 10.0f,
      .recoveryTriggerPeriod = 5 * IMU_LOOP_HZ, /* 5 seconds */
  };
  FusionAhrsSetSettings(&ahrs, &settings);

  error = ImuErrorCode::INIT;
}

int8_t IMU::start() {
  // stdio_init_all();
  i2c_init(&i2c_, speed_);
  gpio_set_function(i2c_sda_, GPIO_FUNC_I2C);
  gpio_set_function(i2c_scl_, GPIO_FUNC_I2C);
  gpio_pull_up(i2c_sda_);
  gpio_pull_up(i2c_scl_);

  return icm20948_init(&config_);

  // sleep_ms(1);

  // for (int i = 0; i < 100; i++) {
  //   imu_.begin(&i2c_, address_, false);
  //   if (imu_.status == ICM_20948_Stat_Ok) {
  //     break;
  //   }
  //   sleep_ms(500);
  // }

  // sleep_ms(1);

  // if (imu_.swReset() != ICM_20948_Stat_Ok) {
  //   // status.setErrorString(imu_.statusString());
  //   // return ImuErrorCode::ERROR;
  //   sleep_ms(1);
  // }

  // sleep_ms(250);

  // // Now wake the sensor up
  // imu_.sleep(false);
  // imu_.lowPower(false);

  // imu_.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  // if (imu_.status != ICM_20948_Stat_Ok) {
  //   // status.setErrorString(imu_.statusString());
  //   // return ImuErrorCode::ERROR;
  //   sleep_ms(1);
  // }

  // // Set full scale ranges for both acc and gyr
  // ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  // myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  //                 // gpm2
  //                 // gpm4
  //                 // gpm8
  //                 // gpm16

  // myFSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
  //                   // dps250
  //                   // dps500
  //                   // dps1000
  //                   // dps2000

  // imu_.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  // if (imu_.status != ICM_20948_Stat_Ok) {
  //   // status.setErrorString(imu_.statusString());
  //   // return ImuErrorCode::ERROR;
  //   sleep_ms(1);
  // }

  // // Set up Digital Low-Pass Filter configuration
  // ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  // myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
  //                                 // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
  //                                 // acc_d111bw4_n136bw
  //                                 // acc_d50bw4_n68bw8
  //                                 // acc_d23bw9_n34bw4
  //                                 // acc_d11bw5_n17bw
  //                                 // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
  //                                 // acc_d473bw_n499bw

  // myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
  //                                   // gyr_d196bw6_n229bw8
  //                                   // gyr_d151bw8_n187bw6
  //                                   // gyr_d119bw5_n154bw3
  //                                   // gyr_d51bw2_n73bw3
  //                                   // gyr_d23bw9_n35bw9
  //                                   // gyr_d11bw6_n17bw8
  //                                   // gyr_d5bw7_n8bw9
  //                                   // gyr_d361bw4_n376bw5

  // imu_.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  // if (imu_.status != ICM_20948_Stat_Ok) {
  //   // status.setErrorString(imu_.statusString());
  //   // return ImuErrorCode::ERROR;
  //   sleep_ms(1);
  // }

  // return 0;

  // // Enable DMP
  // if (imu_.initializeDMP() != ICM_20948_Stat_Ok) {
  //   sleep_ms(1);
  //   // return ImuErrorCode::DMP_FAILED;
  // }

  // sleep_ms(1);

  // // DMP sensor options are defined in ICM_20948_DMP.h
  // //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  // //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  // //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  // //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  // //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  // //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  // //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  // //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  // //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  // //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  // //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  // //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  // //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  // //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  // //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // // Raw Accelerometer
  // // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);

  // // 9 axis quaternion
  // if (imu_.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) != ICM_20948_Stat_Ok) {
  //   sleep_ms(1);
  //   // return ImuErrorCode::DMP_FAILED;
  // }

  // // // Accelerometer
  // // if (imu_.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) != ICM_20948_Stat_Ok) {
  // //   sleep_ms(1);
  // //   return ImuErrorCode::DMP_FAILED;
  // // }

  // // // Gyro
  // // if (imu_.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) != ICM_20948_Stat_Ok) {
  // //   sleep_ms(1);
  // //   return ImuErrorCode::DMP_FAILED;
  // // }

  // // // Compass
  // // if (imu_.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) != ICM_20948_Stat_Ok) {
  // //   sleep_ms(1);
  // //   return ImuErrorCode::DMP_FAILED;
  // // }

  // // Max speed 5hz?
  // // Configuring DMP to output data at multiple ODRs:
  // // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // // Setting value can be calculated as follows:
  // // Value = (DMP running rate / ODR ) - 1
  // // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  // if (imu_.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) != ICM_20948_Stat_Ok) {  // Set to 225Hz
  //   return ImuErrorCode::DMP_FAILED;
  // }

  // // if (imu_.setDMPODRrate(DMP_ODR_Reg_Accel, 0) != ICM_20948_Stat_Ok) {  // Set to 225Hz
  // //   return ImuErrorCode::DMP_FAILED;
  // // }

  // // if (imu_.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) != ICM_20948_Stat_Ok) {  // Set to 225Hz
  // //   return ImuErrorCode::DMP_FAILED;
  // // }

  // // if (imu_.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) != ICM_20948_Stat_Ok) {  // Set to 225Hz
  // //   return ImuErrorCode::DMP_FAILED;
  // // }

  // // Enable FIFO queue
  // if (imu_.enableFIFO() != ICM_20948_Stat_Ok) {
  //   return ImuErrorCode::DMP_FAILED;
  // }

  // // Enable the DMP
  // if (imu_.enableDMP() != ICM_20948_Stat_Ok) {
  //   return ImuErrorCode::DMP_FAILED;
  // }

  // // Reset DMP
  // if (imu_.resetDMP() != ICM_20948_Stat_Ok) {
  //   error = ImuErrorCode::DMP_FAILED;
  // }

  // // Reset FIFO
  // if (imu_.resetFIFO() != ICM_20948_Stat_Ok) {
  //   error = ImuErrorCode::DMP_FAILED;
  // }

  // // TODO: Show status in dialod
  // // SERIAL_PORT.println(myICM.statusString());
  // //   if (myICM.status != ICM_20948_Stat_Ok)

  // // return icm20948_init(&config_);
  // return 0;
}

int8_t IMU::calibrate() {
  // TODO: Calibrate accelerometer and gyro by reading a bunch of times and averaging out to get the bias
  icm20948_cal_gyro(&config_, data_.gyro_bias);
  icm20948_cal_accel(&config_, data_.accel_bias);

  // The magnetometer calibration is a bit more involved, as you need to rotate
  // the sensor around all axes for some period of time. We will take a manual
  // precalculated one for now. icm20948_cal_mag_simple(&config_,
  // data_.mag_bias);

  // imu_.setBiasCPassX(data_.mag_bias[0]);
  // imu_.setBiasCPassY(data_.mag_bias[1]);
  // imu_.setBiasCPassZ(data_.mag_bias[2]);

  return 0;
}

// std::array<float, 3> IMU::quaternianToEuler(const Quaternion& quaternion) {
//   std::array<float, 3> euler;
//   euler[0] = -1.0f * asinf(2.0f * (quaternion.x) * (quaternion.z) +
//                            2.0f * (quaternion.w) * (quaternion.y));
//   euler[1] = atan2f(
//       2.0f * (quaternion.y) * (quaternion.z) - 2.0f * (quaternion.w) * (quaternion.x),
//       2.0f * (quaternion.w) * (quaternion.w) + 2.0f * (quaternion.z) * (quaternion.z) -
//           1.0f);
//   euler[2] = atan2f(
//       2.0f * (quaternion.x) * (quaternion.y) - 2.0f * (quaternion.w) * (quaternion.z),
//       2.0f * (quaternion.w) * (quaternion.w) + 2.0f * (quaternion.x) * (quaternion.x) -
//           1.0f);
//   return euler;
// }

// Convert NED to ENU
// https://answers.ros.org/question/343913/robot_localization-with-imu-and-conventions/
// https://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html
// void IMU:ned_to_enu() {
//   accel_g_[0] = (float)data_.accel_raw[0];
//   accel_g_[1] = (float)data_.accel_raw[1];
//   accel_g_[2] = (float)data_.accel_raw[2];
//   gyro_dps_[0] = (float)data_.gyro_raw[0];
//   gyro_dps_[1] = (float)data_.gyro_raw[1];
//   gyro_dps_[2] = (float)data_.gyro_raw[2];
//   mag_ut_[0] = (float)data_.mag_raw[1];
//   mag_ut_[1] = (float)-data_.mag_raw[0];
//   mag_ut_[2] = (float)-data_.mag_raw[2];
// }

ImuErrorCode IMU::read() {
  icm20948_read_raw_accel(&config_, data_.accel_raw);
  icm20948_read_raw_gyro(&config_, data_.gyro_raw);
  icm20948_read_raw_mag(&config_, data_.mag_raw);
  icm20948_read_temp_c(&config_, &data_.temp_c);

  icm20948_read_cal_accel(&config_, &data_.accel_raw[0], &data_.accel_bias[0]);
  icm20948_read_cal_gyro(&config_, &data_.gyro_raw[0], &data_.gyro_bias[0]);
  icm20948_read_cal_mag(&config_, &data_.mag_raw[0], &data_.mag_bias[0]);
  icm20948_read_temp_c(&config_, &data_.temp_c);

  accel_g_[0] = ((double)data_.accel_raw[0] / 16384.0) * GRAVITY;
  accel_g_[1] = ((double)data_.accel_raw[1] / 16384.0) * GRAVITY;
  accel_g_[2] = ((double)data_.accel_raw[2] / 16384.0) * GRAVITY;
  gyro_dps_[0] = (double)data_.gyro_raw[0] / 131.0;
  gyro_dps_[1] = (double)data_.gyro_raw[1] / 131.0;
  gyro_dps_[2] = (double)data_.gyro_raw[2] / 131.0;
  mag_ut_[0] = (double)data_.mag_raw[1];
  mag_ut_[1] = (double)data_.mag_raw[0];
  mag_ut_[2] = (double)data_.mag_raw[2];

  has_new_data_ = true;

  // ########################################################################
  // Fusion
  // ########################################################################

  // accel(g)   = raw_value / (65535 / full_scale)
  // ex) if full_scale == +-4g then accel = raw_value / (65535 / 8) = raw_value / 8192
  // gyro(dps)  = raw_value / (65535 / full_scale)
  // ex) if full_scale == +-250dps then gyro = raw_value / (65535 / 500) = raw_value / 131
  // mag(uT)    = raw_value / (32752 / 4912) = (approx) (raw_value / 20) * 3
  // temp  = ((raw_value - ambient_temp) / speed_of_sound) + 21

  // Acquire latest sensor data
  const double timestamp = rmw_uros_epoch_nanos();  // replace this with actual gyroscope timestamp

  // degrees/s
  FusionVector gyroscope = {(float)data_.gyro_raw[0] / 131.0f,
                            (float)data_.gyro_raw[1] / 131.0f,
                            (float)data_.gyro_raw[2] / 131.0f};

  // accelerometer data in g
  FusionVector accelerometer = {(float)data_.accel_raw[0] / 16384.0f,
                                (float)data_.accel_raw[1] / 16384.0f,
                                (float)data_.accel_raw[2] / 16384.0f};

  // magnetometer data in arbitrary units
  FusionVector magnetometer = {(float)data_.mag_raw[0],
                               (float)data_.mag_raw[1],
                               (float)data_.mag_raw[2]};

  // Apply calibration
  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
  magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

  // Update gyroscope offset correction algorithm
  gyroscope = FusionOffsetUpdate(&offset, gyroscope);

  // Calculate delta time (in seconds) to account for gyroscope sample clock error
  static double previousTimestamp;
  const float deltaTime = RCUTILS_NS_TO_S(timestamp - previousTimestamp);
  previousTimestamp = timestamp;

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

  orientation_ = FusionAhrsGetQuaternion(&ahrs);

  // Returns the linear acceleration measurement with the 1 g of gravity removed.
  FusionVector accel = FusionAhrsGetLinearAcceleration(&ahrs);
  accel_g_[0] = accel.axis.x;
  accel_g_[1] = accel.axis.y;
  accel_g_[2] = accel.axis.z;

  // TODO: FusionAxesSwap to convert from NED to ENU, instead of manual conversion

  // ########################################################################

  return ImuErrorCode::OK;
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

FusionQuaternion IMU::getOrientation() {
  return orientation_;
}
