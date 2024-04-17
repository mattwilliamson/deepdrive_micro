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

  msg_out_odom->pose.covariance[0] = 0.001;
  msg_out_odom->pose.covariance[4] = 0.001;
  msg_out_odom->pose.covariance[8] = 0.001;

  return 0;
}

void Node::publish_odom() {
  msg_out_odom->header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  msg_out_odom->header.stamp.nanosec = rmw_uros_epoch_nanos();

  // Get the meters traveled for each motor and average it out
  Meters meters = 0;
  for (auto &motor : motors) {
    meters += motor->getTotalMeters();
  }
  meters /= (double)MOTOR_COUNT;

  msg_out_odom->pose.pose.position.x = meters;

  msg_out_odom->pose.pose.orientation.x =
      motors[IDX_MOTOR_FRONT_LEFT]->getTotalMicrometers();
  msg_out_odom->pose.pose.orientation.y =
      motors[IDX_MOTOR_BACK_LEFT]->getTotalMicrometers();
  msg_out_odom->pose.pose.orientation.z =
      motors[IDX_MOTOR_FRONT_RIGHT]->getTotalMicrometers();
  msg_out_odom->pose.pose.orientation.w =
      motors[IDX_MOTOR_BACK_RIGHT]->getTotalMicrometers();

  RCSOFTCHECK(rcl_publish(&publisher_odom, msg_out_odom, NULL));
}