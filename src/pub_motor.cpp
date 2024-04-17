#include "node.hpp"

void Node::init_motor_pub() {
  RCCHECK(rclc_publisher_init_default(
      &publisher_motor, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg,
                                  MecanumDriveControllerState),
      "~/pulses"));
}

void Node::publish_motor() {
  // Publish motor state
  mgs_out_motor.header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  mgs_out_motor.header.stamp.nanosec = rmw_uros_epoch_nanos();

  // 121us to calculate these
  mgs_out_motor.front_left_wheel_velocity =
      motors[IDX_MOTOR_FRONT_LEFT]->getSpeedMeters();
  mgs_out_motor.front_right_wheel_velocity =
      motors[IDX_MOTOR_FRONT_RIGHT]->getSpeedMeters();
  mgs_out_motor.back_left_wheel_velocity =
      motors[IDX_MOTOR_BACK_LEFT]->getSpeedMeters();
  mgs_out_motor.back_right_wheel_velocity =
      motors[IDX_MOTOR_BACK_RIGHT]->getSpeedMeters();

  // 6700us for publisher_motor
  RCSOFTCHECK(rcl_publish(&publisher_motor, &mgs_out_motor, NULL));
}