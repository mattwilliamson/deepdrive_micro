#include "pub_imu.hpp"

PubImu::PubImu(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
               int64_t timer_hz,
               const char *topic_name,
               const char *frame_id) {
  node_ = node;
  allocator_ = allocator;
  support_ = support;
  msg_ = sensor_msgs__msg__Imu__create();
  msg_->header.frame_id = micro_ros_string_utilities_init(frame_id);

  mutex_init(&lock_);

  data_ready_ = false;

  // TODO: Handle status properly

  status_ = imu.start();
  // status.set(Status::Connected);

  // Calibrate the IMU
  status_ = imu.calibrate();
  // TODO: Save the compass calibration to EEPROM or flash or something and
  // expose a service to trigger it

  status_ = rclc_publisher_init_default(
      &publisher_, node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      topic_name);

  if (!add_repeating_timer_us(-MICROSECONDS / timer_hz, PubImu::trigger, NULL, &timer_)) {
    // printf("Failed to add control loop timer\r\n");
    status_ = -1;
  }
}

void PubImu::calculate() {
  if (!_pub_imu_triggered) {
    return;
  }

  mutex_enter_blocking(&lock_);

  imu.read();

  Vector3 accel = imu.getAccel();

  // NED -> ENU conversion
  msg_->linear_acceleration.x = accel.y;
  msg_->linear_acceleration.y = accel.x;
  msg_->linear_acceleration.z = -accel.z;

  msg_->linear_acceleration_covariance[0] = 0.1;
  msg_->linear_acceleration_covariance[4] = 0.1;
  msg_->linear_acceleration_covariance[8] = 0.1;

  Vector3 gyro = imu.getGyro();

  // NED -> ENU conversion
  msg_->angular_velocity.x = gyro.y;
  msg_->angular_velocity.y = gyro.x;
  msg_->angular_velocity.z = -gyro.z;

  msg_->angular_velocity_covariance[0] = 0.1;
  msg_->angular_velocity_covariance[4] = 0.1;
  msg_->angular_velocity_covariance[8] = 0.1;

  Quaternion orientation = imu.getOrientation();

  // NED -> ENU conversion
  msg_->orientation.x = orientation.y;
  msg_->orientation.y = orientation.x;
  msg_->orientation.z = -orientation.z;
  msg_->orientation.w = orientation.w;

  msg_->orientation_covariance[0] = 0.1;
  msg_->orientation_covariance[4] = 0.1;
  msg_->orientation_covariance[8] = 0.1;

  // For testing
  // auto euler = IMU::quaternianToEuler(orientation);
  // msg_->linear_acceleration.x = euler[0];
  // msg_->linear_acceleration.y = euler[1];
  // msg_->linear_acceleration.z = euler[2];

  // msg_->linear_acceleration_covariance[0] = 0.1;
  // msg_->linear_acceleration_covariance[4] = 0.1;
  // msg_->linear_acceleration_covariance[8] = 0.1;

  // Set timestamp
  PubSub::set_timestamp_header(&msg_->header);

  data_ready_ = true;
  _pub_imu_triggered = false;

  // Don't publish a transform. robot_localization will fuse our estimates and do that
  mutex_exit(&lock_);
}

void PubImu::publish() {
  mutex_enter_blocking(&lock_);
  if (data_ready_) {
    status_ = rcl_publish(&publisher_, msg_, NULL);
    data_ready_ = false;
  }
  mutex_exit(&lock_);
}