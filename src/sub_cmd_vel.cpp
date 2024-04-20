#include <node.hpp>

static void subscription_cmd_vel_callback_msg(const void *msgIn) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgIn;
  // Cast received message to used type
  Node::getInstance().subscription_cmd_vel_callback(m);
}

// Subscriber callback for cmd vel
void Node::subscription_cmd_vel_callback(const geometry_msgs__msg__Twist *m) {
  status.set(Status::Active);

  // TODO: Handle cmd_vel timeout
  // const auto age_of_last_command = time - last_command_msg->header.stamp;
  // // Brake if cmd_vel has timeout, override the stored command
  // if (age_of_last_command > cmd_vel_timeout_)
  // {
  //   last_command_msg->twist.linear.x = 0.0;
  //   last_command_msg->twist.angular.z = 0.0;
  // }

  // TODO: handle limits & acceleration
  const double r = (double)Motor::WHEEL_DIAMETER / 2.0 / Motor::MICRO_METERS;
  const double wheel_separation = (double)Motor::WHEEL_BASE / Motor::MICRO_METERS;
  double x = m->linear.x;
  double theta = m->angular.z;

  // Commanded llinear velocity for each motor side in m/s
  double left = (x - theta * wheel_separation / 2.0) / r;
  double right = (x + theta * wheel_separation / 2.0) / r;

  // Set the target speed for each motor
  motors[IDX_MOTOR_FRONT_LEFT]->setTargetSpeedMeters(left);
  motors[IDX_MOTOR_BACK_LEFT]->setTargetSpeedMeters(left);
  motors[IDX_MOTOR_FRONT_RIGHT]->setTargetSpeedMeters(right);
  motors[IDX_MOTOR_BACK_RIGHT]->setTargetSpeedMeters(right);
}

int Node::init_cmd_vel() {
  RCCHECK(rclc_subscription_init_best_effort(
      &subscriber_cmd_vel, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "~/cmd_vel"));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber_cmd_vel, &msg_in_cmd_vel,
      &subscription_cmd_vel_callback_msg, ON_NEW_DATA));

  return 0;
}
