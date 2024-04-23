#include <algorithm>

#include "node.hpp"

int Node::init_odom() {
  mutex_init(&odom_lock);

  RCCHECK(rclc_publisher_init_default(
      &publisher_odom, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "~/odom"));

  // TODO: Make sure we got the frame and everything correct
  msg_out_odom = nav_msgs__msg__Odometry__create();
  msg_out_odom->header.frame_id = micro_ros_string_utilities_init("odom");
  msg_out_odom->child_frame_id = micro_ros_string_utilities_init("base_link");

  // Pose
  msg_out_odom->pose.covariance[0] = 0.001;
  msg_out_odom->pose.covariance[4] = 0.001;
  msg_out_odom->pose.covariance[8] = 0.001;

  msg_out_odom->pose.pose.position.x = 0;
  msg_out_odom->pose.pose.position.y = 0;
  msg_out_odom->pose.pose.position.z = 0;

  msg_out_odom->pose.pose.orientation.x = 0;
  msg_out_odom->pose.pose.orientation.y = 0;
  msg_out_odom->pose.pose.orientation.z = 0;

  // Velocity
  msg_out_odom->twist.covariance[0] = 0.001;
  msg_out_odom->twist.covariance[4] = 0.001;
  msg_out_odom->twist.covariance[8] = 0.001;

  msg_out_odom->twist.twist.linear.x = 0;
  msg_out_odom->twist.twist.linear.y = 0;
  msg_out_odom->twist.twist.linear.z = 0;

  msg_out_odom->twist.twist.angular.x = 0;
  msg_out_odom->twist.twist.angular.y = 0;
  msg_out_odom->twist.twist.angular.z = 0;

  return 0;
}

double Node::ceil_radians(double rad) {
  if (rad > M_PI) {
    return rad - 2 * M_PI;
  } else if (rad < -M_PI) {
    return rad + 2 * M_PI;
  } else {
    return rad;
  }
}

void Node::calculate_odom() {
  mutex_enter_blocking(&odom_lock);
  // TODO: Flag that the data has been processed

  // Do this on core1 to free up cycle time on core0

  // Meters traveled per side since last loop
  Micrometers left = (motors[IDX_MOTOR_FRONT_LEFT]->getMicrometersLoop() +
                      motors[IDX_MOTOR_BACK_LEFT]->getMicrometersLoop()) / 2.0;

  Micrometers right = (motors[IDX_MOTOR_FRONT_RIGHT]->getMicrometersLoop() +
                       motors[IDX_MOTOR_BACK_RIGHT]->getMicrometersLoop()) / 2.0;

  // Calculate the average linear distance to figure where the center of mass has moved
  Micrometers delta_x = (right + left) / 2;

  // Calculate the number of radians the robot has turned since the last loop
  Radians delta_theta = asin((double)(right - left) / Motor::WHEEL_BASE);

  // Get the average angle from the last loop compared to the new angle
  Radians avg_angle = odom_yaw + (delta_theta / 2);

  // Calculate the new pose (x, y, and theta)
  Micrometers translation_x = ((double)cos(avg_angle) * delta_x);
  Micrometers translation_y = ((double)sin(avg_angle) * delta_x);

  // Keep internal state pure micrometers and do conversion as needed
  odom_x += translation_x;
  odom_y += translation_y;

  // TODO: Accessors for these
  msg_out_odom->pose.pose.position.x = (double)odom_x / MICRO_METERS;
  msg_out_odom->pose.pose.position.y = (double)odom_y / MICRO_METERS;

  // Stay within bounds of -pi to pi
  // TODO: Setter for this
  odom_yaw += delta_theta;
  odom_yaw = ceil_radians(odom_yaw);

  // Update the orientation quaternion and normalize it
  // TODO: Function for this
  Quaternion q = Quaternion(0.0, 0.0, odom_yaw);
  q.normalize();

  msg_out_odom->pose.pose.orientation.x = q.x;
  msg_out_odom->pose.pose.orientation.y = q.y;
  msg_out_odom->pose.pose.orientation.z = q.z;
  msg_out_odom->pose.pose.orientation.w = q.w;

  // Update the velocity linear in m/s and angular rad/s from meters/loop and radians/loop
  msg_out_odom->twist.twist.linear.x = (double)delta_x * CONTROL_LOOP_HZ / MICRO_METERS;
  msg_out_odom->twist.twist.angular.z = delta_theta * CONTROL_LOOP_HZ;

  // Set timestamp
  msg_out_odom->header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  msg_out_odom->header.stamp.nanosec = rmw_uros_epoch_nanos();

  // Don't publish a transform. robot_localization will fuse our estimates and do that
  mutex_exit(&odom_lock);
}

void Node::publish_odom() {
  mutex_enter_blocking(&odom_lock);
  RCSOFTCHECK(rcl_publish(&publisher_odom, msg_out_odom, NULL));
  mutex_exit(&odom_lock);
}