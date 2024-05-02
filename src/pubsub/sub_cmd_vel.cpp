#include "pubsub/sub_cmd_vel.hpp"

SubCmdVel *_sub_cmd_vel_instance = nullptr;

// Event handler needs a static callback
static void _sub_cmd_vel_callback(const void *msgIn) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgIn;
  // Cast received message to used type
  if (_sub_cmd_vel_instance != nullptr) {
    _sub_cmd_vel_instance->callback(m);
  }
}

SubCmdVel::SubCmdVel(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
                     rclc_executor_t *executor, MotorManager *motor_manager, const char *topic) {
  _sub_cmd_vel_instance = this;
  node_ = node;
  allocator_ = allocator;
  support_ = support;
  motor_manager_ = motor_manager;
  executor_ = executor;
  msg_ = geometry_msgs__msg__Twist__create();
  prev_msg_ = geometry_msgs__msg__Twist__create();
  status_ = 0;
  data_ready_ = false;
  topic_ = topic;
  last_message_time_ = 0;

  prev_msg_->linear.x = 0;
  prev_msg_->angular.z = 0;

  status_ = rclc_subscription_init_default(&subscription_, node_,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                           topic_);
  status_ = rclc_executor_add_subscription(executor_, &subscription_, msg_,
                                           &_sub_cmd_vel_callback, ON_NEW_DATA);
}

// Subscriber callback for cmd vel
void SubCmdVel::callback(const geometry_msgs__msg__Twist *msg) {
  // status.set(Status::Active);

  // TODO: handle limits & acceleration

  // Calculate the target speed for each motor

  const double wheel_separation = (double)Motor::WHEEL_BASE / MICRO_METERS;
  // const double radius = (double)Motor::WHEEL_RADIUS / MICRO_METERS;
  double x = msg->linear.x;
  double z = msg->angular.z;

  // Limit the acceleration with MAX_ACCELERATION_LINEAR and MAX_ACCELERATION_ROTATION
  int64_t elapsed_ns = rmw_uros_epoch_nanos() - last_message_time_;
  double delta_x = x - prev_msg_->linear.x;
  double delta_z = z - prev_msg_->angular.z;
  double max_lin = MAX_ACCELERATION_LINEAR * (elapsed_ns / 1e9);
  double max_rot = MAX_ACCELERATION_ROTATION * (elapsed_ns / 1e9);

  if (delta_x > max_lin) {
    x = prev_msg_->linear.x + max_lin;
  } else if (delta_x < -max_lin) {
    x = prev_msg_->linear.x - max_lin;
  }

  if (delta_z > max_rot) {
    z = prev_msg_->angular.z + max_rot;
  } else if (delta_z < -max_rot) {
    z = prev_msg_->angular.z - max_rot;
  }

  // Commanded linear velocity for each motor side in m/s
  // double left = (x - (cos(z) * wheel_separation / 2.0));
  // double right = (x + (sin(z) * wheel_separation / 2.0));
  double left = x - (wheel_separation * z);
  double right = x + (wheel_separation * z);

  auto motors = motor_manager_->get_motors();

  // Set the target speed for each motor
  motors[IDX_MOTOR_FRONT_LEFT]->setTargetSpeedMeters(left);
  motors[IDX_MOTOR_BACK_LEFT]->setTargetSpeedMeters(left);
  motors[IDX_MOTOR_FRONT_RIGHT]->setTargetSpeedMeters(right);
  motors[IDX_MOTOR_BACK_RIGHT]->setTargetSpeedMeters(right);

  last_message_time_ = rmw_uros_epoch_nanos();
  prev_msg_->linear.x = msg->linear.x;
  prev_msg_->angular.z = msg->angular.z;
  
}

SubCmdVel::~SubCmdVel() {
  // if (sublisher_ != nullptr) {
  status_ = rcl_subscription_fini(&subscription_, node_);
  // }
}