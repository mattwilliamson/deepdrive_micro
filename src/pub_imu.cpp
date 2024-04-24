#include "node.hpp"

int Node::init_imu() {
#ifdef IMU_ENABLED
  mutex_init(&imu_lock);

  RCCHECK(rclc_publisher_init_default(
      &publisher_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "~/imu"));

  RCCHECK(rclc_publisher_init_default(
      &publisher_mag, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
      "~/mag"));

  msg_out_imu = sensor_msgs__msg__Imu__create();
  msg_out_imu->header.frame_id = micro_ros_string_utilities_init(IMU_FRAME);

  msg_out_mag = sensor_msgs__msg__MagneticField__create();
  msg_out_mag->header.frame_id = micro_ros_string_utilities_init(IMU_FRAME);

  int8_t imu_status = -1;

  // Try to connect to the IMU 20 times
  for (int i = 0; i < 20; i++) {
    imu_status = imu.start();
    if (imu_status == 0) {
      status.set(Status::Connected);

      // Calibrate the IMU
      imu.calibrate();
      // TODO: Save the compass calibration to EEPROM or flash or something and
      // expose a service to trigger it
      return 0;
    } else {
      status.set(Status::ErrorIMU);
      const char * imu_status_string = imu.statusString();
      status.setErrorString(imu_status_string);
      // printf("IMU start failed\r\n");
      sleep_ms(1000);
    }
  }

#endif

  return imu_status;
}

void Node::publish_imu() {
#ifdef IMU_ENABLED
  mutex_enter_blocking(&imu_lock);

  Vector3 accel = imu.getAccel();

  // NED -> ENU conversion
  msg_out_imu->linear_acceleration.x = accel.y;
  msg_out_imu->linear_acceleration.y = accel.x;
  msg_out_imu->linear_acceleration.z = -accel.z;

  msg_out_imu->linear_acceleration_covariance[0] = 0.1;
  msg_out_imu->linear_acceleration_covariance[4] = 0.1;
  msg_out_imu->linear_acceleration_covariance[8] = 0.1;

  Vector3 gyro = imu.getGyro();

  // NED -> ENU conversion
  msg_out_imu->angular_velocity.x = gyro.y;
  msg_out_imu->angular_velocity.y = gyro.x;
  msg_out_imu->angular_velocity.z = -gyro.z;

  msg_out_imu->angular_velocity_covariance[0] = 0.1;
  msg_out_imu->angular_velocity_covariance[4] = 0.1;
  msg_out_imu->angular_velocity_covariance[8] = 0.1;

  Quaternion orientation = imu.getOrientation();

// NED -> ENU conversion
  msg_out_imu->orientation.x = orientation.y;
  msg_out_imu->orientation.y = orientation.x;
  msg_out_imu->orientation.z = -orientation.z;
  msg_out_imu->orientation.w = orientation.w;

  msg_out_imu->orientation_covariance[0] = 0.1;
  msg_out_imu->orientation_covariance[4] = 0.1;
  msg_out_imu->orientation_covariance[8] = 0.1;

  // For testing
  // auto euler = IMU::quaternianToEuler(orientation);
  // msg_out_imu->linear_acceleration.x = euler[0];
  // msg_out_imu->linear_acceleration.y = euler[1];
  // msg_out_imu->linear_acceleration.z = euler[2];

  // msg_out_imu->linear_acceleration_covariance[0] = 0.1;
  // msg_out_imu->linear_acceleration_covariance[4] = 0.1;
  // msg_out_imu->linear_acceleration_covariance[8] = 0.1;
  

  mutex_exit(&imu_lock);

  PubSub::set_timestamp_header(&msg_out_imu->header);
  
  // RCSOFTCHECK(rcl_publish(&publisher_imu, msg_out_imu, NULL));
  rcl_ret_t pub_ok = rcl_publish(&publisher_imu, msg_out_imu, NULL);

  if (pub_ok != RCL_RET_OK) {
    printf("Error publishing IMU data\r\n");
  }

  // Magnetometer
  // Vector3 mag = imu.getMag();

  // msg_out_mag->magnetic_field.x = mag.x;
  // msg_out_mag->magnetic_field.y = mag.y;
  // msg_out_mag->magnetic_field.z = mag.z;

  // msg_out_mag->magnetic_field_covariance[0] = 0.1;
  // msg_out_mag->magnetic_field_covariance[4] = 0.1;
  // msg_out_mag->magnetic_field_covariance[8] = 0.1;

  // set_timestamp_header(&msg_out_mag->header);

  // RCSOFTCHECK(rcl_publish(&publisher_mag, &msg_out_mag, NULL));
#endif
}