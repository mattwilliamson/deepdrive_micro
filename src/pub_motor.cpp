#include "node.hpp"

void Node::init_motor_pub() {
  RCCHECK(rclc_publisher_init_default(
      &publisher_motor_speed, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, MecanumDriveControllerState),
      "~/pulses/out"));
  RCCHECK(rclc_publisher_init_default(
      &publisher_motor_cmd, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, MecanumDriveControllerState),
      "~/pulses/cmd"));
}

void Node::publish_motor() {
  // Publish motor state
  // 121us to calculate these
  // TODO: Do calculation in control loop and publish here
  mgs_out_motor_speed.header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  mgs_out_motor_speed.header.stamp.nanosec = rmw_uros_epoch_nanos();
  mgs_out_motor_speed.front_left_wheel_velocity = motors[IDX_MOTOR_FRONT_LEFT]->getSpeedMeters();
  mgs_out_motor_speed.front_right_wheel_velocity = motors[IDX_MOTOR_FRONT_RIGHT]->getSpeedMeters();
  mgs_out_motor_speed.back_left_wheel_velocity = motors[IDX_MOTOR_BACK_LEFT]->getSpeedMeters();
  mgs_out_motor_speed.back_right_wheel_velocity = motors[IDX_MOTOR_BACK_RIGHT]->getSpeedMeters();

// 6700us for publisher_motor
  RCSOFTCHECK(rcl_publish(&publisher_motor_speed, &mgs_out_motor_speed, NULL));

  // Publish motor state for setpoint speed
  // 121us to calculate these
  // TODO: Do calculation in control loop and publish here
  mgs_out_motor_cmd.header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  mgs_out_motor_cmd.header.stamp.nanosec = rmw_uros_epoch_nanos();
  mgs_out_motor_cmd.front_left_wheel_velocity = motors[IDX_MOTOR_FRONT_LEFT]->getTargetSpeedMeters();
  mgs_out_motor_cmd.front_right_wheel_velocity = motors[IDX_MOTOR_FRONT_RIGHT]->getTargetSpeedMeters();
  mgs_out_motor_cmd.back_left_wheel_velocity = motors[IDX_MOTOR_BACK_LEFT]->getTargetSpeedMeters();
  mgs_out_motor_cmd.back_right_wheel_velocity = motors[IDX_MOTOR_BACK_RIGHT]->getTargetSpeedMeters();

  // 6700us for publisher_motor
  RCSOFTCHECK(rcl_publish(&publisher_motor_cmd, &mgs_out_motor_cmd, NULL));
}