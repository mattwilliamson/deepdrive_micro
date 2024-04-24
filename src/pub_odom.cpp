#include "pub_odom.hpp"

PubOdom::PubOdom(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
                 std::vector<Motor *> motors,
                 int64_t timer_hz,
                 const char *topic_name, const char *frame_id, const char *child_frame_id) {
  node_ = node;
  allocator_ = allocator;
  support_ = support;
  motors_ = motors;
  msg_ = nav_msgs__msg__Odometry__create();

  mutex_init(&lock_);

  data_ready_ = false;

  // Pose
  msg_->pose.covariance[0] = 0.5;
  msg_->pose.covariance[7] = 0.5;
  msg_->pose.covariance[14] = 0.5;
  msg_->pose.covariance[21] = 0.5;
  msg_->pose.covariance[28] = 0.5;
  msg_->pose.covariance[35] = 0.5;

  msg_->pose.pose.position.x = 0;
  msg_->pose.pose.position.y = 0;
  msg_->pose.pose.position.z = 0;

  msg_->pose.pose.orientation.x = 0;
  msg_->pose.pose.orientation.y = 0;
  msg_->pose.pose.orientation.z = 0;

  // Velocity
  msg_->twist.covariance[0] = 0.1;
  msg_->twist.covariance[4] = 0.1;
  msg_->twist.covariance[8] = 0.1;

  msg_->twist.twist.linear.x = 0;
  msg_->twist.twist.linear.y = 0;
  msg_->twist.twist.linear.z = 0;

  msg_->twist.twist.angular.x = 0;
  msg_->twist.twist.angular.y = 0;
  msg_->twist.twist.angular.z = 0;
  
  status_ = rclc_publisher_init_default(
      &publisher_, node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      topic_name);

  msg_->header.frame_id = micro_ros_string_utilities_init(frame_id);
  msg_->child_frame_id = micro_ros_string_utilities_init(child_frame_id);

  if (!add_repeating_timer_us(-MICROSECONDS / timer_hz, PubOdom::trigger, NULL, &timer_)) {
    // printf("Failed to add control loop timer\r\n");
    status_ = -1;
  }
}

void PubOdom::calculate() {
  mutex_enter_blocking(&lock_);
  // TODO: Flag that the data has been processed

  // Do this on core1 to free up cycle time on core0

  // Meters traveled per side since last loop
  Micrometers left = (motors_[IDX_MOTOR_FRONT_LEFT]->getMicrometersLoop() +
                      motors_[IDX_MOTOR_BACK_LEFT]->getMicrometersLoop()) /
                     2.0;

  Micrometers right = (motors_[IDX_MOTOR_FRONT_RIGHT]->getMicrometersLoop() +
                       motors_[IDX_MOTOR_BACK_RIGHT]->getMicrometersLoop()) /
                      2.0;

  // Calculate the average linear distance to figure where the center of mass has moved
  Micrometers delta_x = (right + left) / 2;

  // Calculate the number of radians the robot has turned since the last loop
  Radians delta_theta = asin((double)(right - left) / Motor::WHEEL_BASE);

  // Get the average angle from the last loop compared to the new angle
  Radians avg_angle = yaw_ + (delta_theta / 2);

  // Calculate the new pose (x, y, and theta)
  Micrometers translation_x = ((double)cos(avg_angle) * delta_x);
  Micrometers translation_y = ((double)sin(avg_angle) * delta_x);

  // Keep internal state pure micrometers and do conversion as needed
  x_ += translation_x;
  y_ += translation_y;

  // TODO: Accessors for these
  msg_->pose.pose.position.x = (double)x_ / MICRO_METERS;
  msg_->pose.pose.position.y = (double)y_ / MICRO_METERS;

  // Stay within bounds of -pi to pi
  // TODO: Setter for this
  yaw_ += delta_theta;
  yaw_ = ceil_radians(yaw_);

  // Update the orientation quaternion and normalize it
  // TODO: Function for this
  Quaternion q = Quaternion(0.0, 0.0, yaw_);
  q.normalize();

  msg_->pose.pose.orientation.x = q.x;
  msg_->pose.pose.orientation.y = q.y;
  msg_->pose.pose.orientation.z = q.z;
  msg_->pose.pose.orientation.w = q.w;

  // Update the velocity linear in m/s and angular rad/s from meters/loop and radians/loop
  msg_->twist.twist.linear.x = (double)delta_x * CONTROL_LOOP_HZ / MICRO_METERS;
  msg_->twist.twist.angular.z = delta_theta * CONTROL_LOOP_HZ;

  // Set timestamp
  PubSub::set_timestamp_header(&msg_->header);

  data_ready_ = true;

  // Don't publish a transform. robot_localization will fuse our estimates and do that
  mutex_exit(&lock_);
}

void PubOdom::publish() {
  mutex_enter_blocking(&lock_);
  if (_pub_odom_triggered && data_ready_) {
    status_ = rcl_publish(&publisher_, msg_, NULL);
    data_ready_ = false;
    _pub_odom_triggered = false;
  }
  mutex_exit(&lock_);
}