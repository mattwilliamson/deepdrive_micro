#include "node.hpp"

int Node::init_imu() {
#ifdef IMU_ENABLED
  RCCHECK(rclc_publisher_init_default(
      &publisher_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "~/imu"));

  RCCHECK(rclc_publisher_init_default(
      &publisher_mag, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
      "~/mag"));

  if (imu.start() != 0) {
    printf("IMU start failed\r\n");
    return 1;
  }

  // Calibrate the IMU
  imu.calibrate();
  // TODO: Save the compass calibration to EEPROM or flash or something and
  // expose a service to trigger it

  msg_out_imu = sensor_msgs__msg__Imu__create();
  msg_out_imu->header.frame_id = micro_ros_string_utilities_init(IMU_FRAME);

  msg_out_mag = sensor_msgs__msg__MagneticField__create();
  msg_out_mag->header.frame_id = micro_ros_string_utilities_init(IMU_FRAME);

#endif

  return 0;
}

void Node::publish_imu() {
#ifdef IMU_ENABLED
  int8_t success = imu.read();

  Vector3 accel = imu.getAccel();

  msg_out_imu->linear_acceleration.x = accel.x;
  msg_out_imu->linear_acceleration.y = accel.y;
  msg_out_imu->linear_acceleration.z = accel.z;

  msg_out_imu->linear_acceleration_covariance[0] = 0.001;
  msg_out_imu->linear_acceleration_covariance[4] = 0.001;
  msg_out_imu->linear_acceleration_covariance[8] = 0.001;

  Vector3 gyro = imu.getGyro();

  msg_out_imu->angular_velocity.x = gyro.x;
  msg_out_imu->angular_velocity.y = gyro.y;
  msg_out_imu->angular_velocity.z = gyro.z;

  msg_out_imu->angular_velocity_covariance[0] = 0.001;
  msg_out_imu->angular_velocity_covariance[4] = 0.001;
  msg_out_imu->angular_velocity_covariance[8] = 0.001;

  Quaternion orientation = imu.getOrientation();

  msg_out_imu->orientation.x = orientation.x;
  msg_out_imu->orientation.y = orientation.y;
  msg_out_imu->orientation.z = orientation.z;
  msg_out_imu->orientation.w = orientation.w;

  msg_out_imu->orientation_covariance[0] = 0.001;
  msg_out_imu->orientation_covariance[4] = 0.001;
  msg_out_imu->orientation_covariance[8] = 0.001;

  msg_out_imu->header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  msg_out_imu->header.stamp.nanosec = rmw_uros_epoch_nanos();

  RCSOFTCHECK(rcl_publish(&publisher_imu, msg_out_imu, NULL));

  // Magnetometer
  Vector3 mag = imu.getMag();

  msg_out_mag->magnetic_field.x = mag.x;
  msg_out_mag->magnetic_field.y = mag.y;
  msg_out_mag->magnetic_field.z = mag.z;

  msg_out_mag->magnetic_field_covariance[0] = 0.001;
  msg_out_mag->magnetic_field_covariance[4] = 0.001;
  msg_out_mag->magnetic_field_covariance[8] = 0.001;

  msg_out_mag->header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  msg_out_mag->header.stamp.nanosec = rmw_uros_epoch_nanos();

  RCSOFTCHECK(rcl_publish(&publisher_mag, msg_out_mag, NULL));
#endif
}