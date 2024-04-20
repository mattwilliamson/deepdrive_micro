#include "node.hpp"

int Node::init_odom() {
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


void Node::calculate_odom() {
  // Do this on core1 to free up cycle time on core0
  // TODO: mutex here

  // Time elapsed in seconds since the last loop
  const double step_time = 1.0 / CONTROL_LOOP_HZ;

  // x, y, theta
  static double pose[3] = {0.0, 0.0, 0.0};

  // Get the meters/s and average it out for each side since the last loop
  Meters meters[2] = {0, 0};

  meters[MOTOR_LEFT] = motors[IDX_MOTOR_FRONT_LEFT]->getSpeedMeters() +
                       motors[IDX_MOTOR_BACK_LEFT]->getSpeedMeters() /
                           MOTOR_COUNT / 2.0;

  meters[MOTOR_RIGHT] = motors[IDX_MOTOR_FRONT_RIGHT]->getSpeedMeters() +
                        motors[IDX_MOTOR_BACK_RIGHT]->getSpeedMeters() /
                            MOTOR_COUNT / 2.0;

  // Get the radians/s and average it out for each side since the last loop
  Radians radians[2] = {0, 0};

  radians[MOTOR_LEFT] = motors[IDX_MOTOR_FRONT_LEFT]->getSpeedRadians() +
                        motors[IDX_MOTOR_BACK_LEFT]->getSpeedRadians() /
                            MOTOR_COUNT / 2.0;

  radians[MOTOR_RIGHT] = motors[IDX_MOTOR_FRONT_RIGHT]->getSpeedRadians() +
                         motors[IDX_MOTOR_BACK_RIGHT]->getSpeedRadians() /
                             MOTOR_COUNT / 2.0;

  const double r = Motor::WHEEL_DIAMETER / 2.0;
  double delta_s = r * (radians[1] + radians[0]) / 2.0 / Motor::MICRO_METERS;
  double delta_theta = r * (radians[1] - radians[0]) / ((double)Motor::WHEEL_BASE / Motor::MICRO_METERS);

  // x, y position in meters, theta in radians
  pose[0] += delta_s * cos(pose[2] + (delta_theta / 2.0));
  pose[1] += delta_s * sin(pose[2] + (delta_theta / 2.0));
  pose[2] += delta_theta;

  // Update the pose, x, y
  msg_out_odom->pose.pose.position.x = pose[0];
  msg_out_odom->pose.pose.position.y = pose[1];

  // Update the orientation quaternion and normalize it
  Quaternion q = Quaternion(0.0, 0.0, delta_theta);
  q.normalize();

  msg_out_odom->pose.pose.orientation.x = q.x;
  msg_out_odom->pose.pose.orientation.y = q.y;
  msg_out_odom->pose.pose.orientation.z = q.z;
  msg_out_odom->pose.pose.orientation.w = q.w;

  // Update the velocity linear m/s and angular rad/s
  msg_out_odom->twist.twist.linear.x = delta_s / step_time;
  msg_out_odom->twist.twist.angular.z = delta_theta / step_time;

  msg_out_odom->header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  msg_out_odom->header.stamp.nanosec = rmw_uros_epoch_nanos();

  // Don't publish a transform. robot_localization will fuse our estimates and do that
}

void Node::publish_odom() {
  // TODO: mutex
  RCSOFTCHECK(rcl_publish(&publisher_odom, msg_out_odom, NULL));
}