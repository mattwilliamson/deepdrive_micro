#include "pubsub/pub_wheel_speed.hpp"

static bool _pub_wheelspeed_triggered;

bool PubWheelSpeed::trigger(repeating_timer_t *rt) {
  _pub_wheelspeed_triggered = true;
  return true;
}

PubWheelSpeed::PubWheelSpeed(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
                             MotorManager *motor_manager,
                             int64_t timer_hz,
                             const char *topic_name,
                             const char *frame_id) {
  node_ = node;
  allocator_ = allocator;
  support_ = support;
  msg_ = control_msgs__msg__MecanumDriveControllerState__create();
  msg_->header.frame_id = micro_ros_string_utilities_init(frame_id);
  motor_manager_ = motor_manager;

  mutex_init(&lock_);

  data_ready_ = false;

  status_ = rclc_publisher_init_default(
      &publisher_, node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, MecanumDriveControllerState),
      topic_name);

  assert(add_repeating_timer_us(-MICROSECONDS / timer_hz, PubWheelSpeed::trigger, NULL, &timer_));
}

void PubWheelSpeed::calculate() {
  if (!_pub_wheelspeed_triggered) {
    return;
  }

  mutex_enter_blocking(&lock_);

  auto motors = motor_manager_->get_motors();

  msg_->front_left_wheel_velocity = motors[IDX_MOTOR_FRONT_LEFT]->getSpeedMeters();
  msg_->front_right_wheel_velocity = motors[IDX_MOTOR_FRONT_RIGHT]->getSpeedMeters();
  msg_->back_left_wheel_velocity = motors[IDX_MOTOR_BACK_LEFT]->getSpeedMeters();
  msg_->back_right_wheel_velocity = motors[IDX_MOTOR_BACK_RIGHT]->getSpeedMeters();

  // msg_->front_left_wheel_velocity = motors[IDX_MOTOR_FRONT_LEFT]->getPulses();
  // msg_->front_right_wheel_velocity = motors[IDX_MOTOR_FRONT_RIGHT]->getPulses();
  // msg_->back_left_wheel_velocity = motors[IDX_MOTOR_BACK_LEFT]->getPulses();
  // msg_->back_right_wheel_velocity = motors[IDX_MOTOR_BACK_RIGHT]->getPulses();

  // Set timestamp
  PubSub::set_timestamp_header(&msg_->header);

  data_ready_ = true;
  _pub_wheelspeed_triggered = false;

  // Don't publish a transform. robot_localization will fuse our estimates and do that
  mutex_exit(&lock_);
}

void PubWheelSpeed::publish() {
  mutex_enter_blocking(&lock_);
  if (data_ready_) {
    status_ = rcl_publish(&publisher_, msg_, NULL);
    data_ready_ = false;
  }
  mutex_exit(&lock_);
}

PubWheelSpeed::~PubWheelSpeed() {
  cancel_repeating_timer(&timer_);
  status_ = rcl_publisher_fini(&publisher_, node_);
}