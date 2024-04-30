#include "pubsub/sub_wheel_speed.hpp"

SubWheelSpeed *_sub_wheel_speed_instance = nullptr;

// Event handler needs a static callback
static void _sub_wheel_speed_callback(const void *msgIn) {
  const control_msgs__msg__MecanumDriveControllerState *m = (const control_msgs__msg__MecanumDriveControllerState *)msgIn;
  // Cast received message to used type
  if (_sub_wheel_speed_instance != nullptr) {
    _sub_wheel_speed_instance->callback(m);
  }
}

SubWheelSpeed::SubWheelSpeed(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
                             rclc_executor_t *executor, MotorManager *motor_manager, const char *topic) {
  _sub_wheel_speed_instance = this;
  node_ = node;
  allocator_ = allocator;
  support_ = support;
  motor_manager_ = motor_manager;
  executor_ = executor;
  status_ = 0;
  data_ready_ = false;
  topic_ = topic;
  msg_ = control_msgs__msg__MecanumDriveControllerState__create();

  status_ = rclc_subscription_init_default(&subscription_, node_,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, MecanumDriveControllerState),
                                           topic_);
  status_ = rclc_executor_add_subscription(executor_, &subscription_, msg_,
                                           &_sub_wheel_speed_callback, ON_NEW_DATA);
  assert(status_ == RCL_RET_OK);
}

// Subscriber callback for cmd vel
void SubWheelSpeed::callback(const control_msgs__msg__MecanumDriveControllerState *msg) {
  // status.set(Status::Active);

  auto motors = motor_manager_->get_motors();

  // Set the target speed for each motor
  motors[IDX_MOTOR_FRONT_LEFT]->setTargetSpeedMeters(msg->front_left_wheel_velocity);
  motors[IDX_MOTOR_BACK_LEFT]->setTargetSpeedMeters(msg->back_left_wheel_velocity);
  motors[IDX_MOTOR_FRONT_RIGHT]->setTargetSpeedMeters(msg->front_right_wheel_velocity);
  motors[IDX_MOTOR_BACK_RIGHT]->setTargetSpeedMeters(msg->back_right_wheel_velocity);
}

SubWheelSpeed::~SubWheelSpeed() {
  // if (sublisher_ != nullptr) {
  status_ = rcl_subscription_fini(&subscription_, node_);
  // }
}