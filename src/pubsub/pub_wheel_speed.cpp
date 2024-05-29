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
  msg_ = sensor_msgs__msg__JointState__create();
  msg_->header.frame_id = micro_ros_string_utilities_init(frame_id);
  motor_manager_ = motor_manager;

  msg_->name = *rosidl_runtime_c__String__Sequence__create(MOTOR_COUNT);
  rosidl_runtime_c__float64__Sequence__init(&msg_->position, MOTOR_COUNT);
  rosidl_runtime_c__float64__Sequence__init(&msg_->velocity, MOTOR_COUNT);
  rosidl_runtime_c__float64__Sequence__init(&msg_->effort, MOTOR_COUNT);

  msg_->name.data[IDX_MOTOR_FRONT_LEFT] = micro_ros_string_utilities_init(MOTOR_JOIN_FRONT_LEFT);
  msg_->name.data[IDX_MOTOR_BACK_LEFT] = micro_ros_string_utilities_init(MOTOR_JOIN_BACK_LEFT);
  msg_->name.data[IDX_MOTOR_FRONT_RIGHT] = micro_ros_string_utilities_init(MOTOR_JOIN_FRONT_RIGHT);
  msg_->name.data[IDX_MOTOR_BACK_RIGHT] = micro_ros_string_utilities_init(MOTOR_JOIN_BACK_RIGHT);
  mutex_init(&lock_);

  data_ready_ = false;

  status_ = rclc_publisher_init_default(
      &publisher_, node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      topic_name);

  assert(add_repeating_timer_us(-MICROSECONDS / timer_hz, PubWheelSpeed::trigger, NULL, &timer_));
}

void PubWheelSpeed::calculate() {
  if (!_pub_wheelspeed_triggered) {
    return;
  }

  mutex_enter_blocking(&lock_);

  auto motors = motor_manager_->get_motors();

  for (int i = 0; i < MOTOR_COUNT; i++) {
    msg_->effort.data[i] = motors[i]->getSpeed();
    msg_->velocity.data[i] = motors[i]->getSpeedMeters();
    msg_->position.data[i] = motors[i]->getPulses();
  }

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