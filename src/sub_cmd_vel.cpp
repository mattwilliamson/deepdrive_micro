#include <node.hpp>

static void subscription_cmd_vel_callback_msg(const void *msgIn) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgIn;
  Node::getInstance().subscription_cmd_vel_callback(m);
}

// Subscriber callback for cmd vel
void Node::subscription_cmd_vel_callback(const geometry_msgs__msg__Twist *m) {
  status.set(Status::Active);

  // Cast received message to used type

  for (auto &motor : motors) {
    motor->setTargetSpeedMeters(m->linear.x);
  }
}

int Node::init_cmd_vel() {
  RCCHECK(rclc_subscription_init_best_effort(
      &subscriber_cmd_vel, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "deepdrive_micro/cmd_vel"));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber_cmd_vel, &msg_in_cmd_vel,
      &subscription_cmd_vel_callback_msg, ON_NEW_DATA));

  return 0;
}
