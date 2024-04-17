#include "node.hpp"

static void subscription_motor_callback_msg(const void *msgIn) {
  const control_msgs__msg__MecanumDriveControllerState *m =
      (const control_msgs__msg__MecanumDriveControllerState *)msgIn;
  Node::getInstance().subscription_motor_callback(m);
}

int Node::init_sub_motor() {
  RCCHECK(rclc_subscription_init_best_effort(
      &subscriber_motor, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg,
                                  MecanumDriveControllerState),
      "deepdrive_micro/cmd"));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber_motor, &msg_in_motor,
      &subscription_motor_callback_msg, ON_NEW_DATA));

  return 0;
}

// Subscriber callback
void Node::subscription_motor_callback(
    const control_msgs__msg__MecanumDriveControllerState *m) {
  status.set(Status::Active);

  // Cast received message to used type

  motors[IDX_MOTOR_FRONT_LEFT]->setTargetSpeedMeters(
      m->front_left_wheel_velocity);
  motors[IDX_MOTOR_FRONT_RIGHT]->setTargetSpeedMeters(
      m->front_right_wheel_velocity);
  motors[IDX_MOTOR_BACK_LEFT]->setTargetSpeedMeters(
      m->back_left_wheel_velocity);
  motors[IDX_MOTOR_BACK_RIGHT]->setTargetSpeedMeters(
      m->back_right_wheel_velocity);
}
