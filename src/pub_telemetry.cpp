#include "pub_telemetry.hpp"

PubTelemetry::PubTelemetry(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
                           MotorManager *motor_manager,
                           int64_t timer_hz,
                           const char *topic_name) {
  node_ = node;
  allocator_ = allocator;
  support_ = support;
  motor_manager_ = motor_manager;
  msg_ = diagnostic_msgs__msg__DiagnosticArray__create();

  mutex_init(&lock_);

  data_ready_ = false;

  status_ = rclc_publisher_init_default(
      &publisher_, node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
      topic_name);

  msg_->header.frame_id = micro_ros_string_utilities_init(DIAGNOSTIC_FRAME);
  diagnostic_msgs__msg__DiagnosticStatus__Sequence__init(&msg_->status, 1);

  msg_->status.data[0].hardware_id = micro_ros_string_utilities_init("deepdrive_micro");
  msg_->status.data[0].name = micro_ros_string_utilities_init("DeepDrive Status");

  diagnostic_msgs__msg__KeyValue__Sequence__init(
      &msg_->status.data[0].values, 2);

  msg_->status.data[0].values.data[Diagnostics::CORE_0].key =
      micro_ros_string_utilities_init("Core 0 Loop Time (us)");

  msg_->status.data[0].values.data[Diagnostics::CORE_1].key =
      micro_ros_string_utilities_init("Core 1 Loop Time (us)");

  if (!add_repeating_timer_us(-MICROSECONDS / timer_hz, PubTelemetry::trigger, NULL, &timer_)) {
    // printf("Failed to add control loop timer\r\n");
    status_ = -1;
  }
}

void PubTelemetry::calculate() {
  if (!_pub_telemetry_triggered) {
    return;
  }

  mutex_enter_blocking(&lock_);

//   auto motors = motor_manager_->get_motors();

  const int timeout_ms = 1000;
  const int attempts = 1;

  rmw_ret_t error_code = rmw_uros_ping_agent(timeout_ms, attempts);

  // TODO: Probably want to check other criteria when disabling motors
  // TODO: Check if two motors on the same side have similar speeds
  // TODO: Check fore multiple ping failures with shorter timeout
  // TODO: This was failing on and off
  // if (error_code != RMW_RET_OK) {
  //   StatusManager::getInstance().set(Status::Error);
  //   motor_manager_->disable_motors();
  // } else {
    // StatusManager::getInstance().set(Status::Error);
    motor_manager_->enable_motors();
  // }

  // Convert core_elapsed[0] to string
  std::string core_elapsed_0_str = std::to_string(core_elapsed[0]);
  msg_->status.data[0].values.data[Diagnostics::CORE_0].value.data =
      const_cast<char *>(core_elapsed_0_str.c_str());

  // Convert core_elapsed[1] to string
  std::string core_elapsed_1_str = std::to_string(core_elapsed[1]);
  msg_->status.data[0].values.data[Diagnostics::CORE_1].value.data =
      const_cast<char *>(core_elapsed_1_str.c_str());

  // IMU Status
  //   const char *imuStatus = imu.statusString();
  //   msg_->status.data[0].values.data[Diagnostics::IMU].value.data =
  //       const_cast<char*>(imuStatus);

  // TODO: set error string from status
  msg_->status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__OK;
  msg_->status.data[0].message = micro_ros_string_utilities_init("OK");

  // TODO: Set status string
  switch (StatusManager::getInstance().get()) {
    case Status::Connecting:
      msg_->status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
      break;
    case Status::Connected:
      msg_->status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__OK;
      break;
    case Status::Active:
      msg_->status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__OK;
      break;
    case Status::Error:
      msg_->status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
      msg_->status.data[0].message = micro_ros_string_utilities_init("Error");
      break;
    case Status::Rebooted:
      msg_->status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
      break;
  }

  // TODO: Send Status
  if (!rmw_uros_epoch_synchronized()) {
    msg_->status.data[0].message = micro_ros_string_utilities_init("Time not synchronized");
  }

  // Set timestamp
  PubSub::set_timestamp_header(&msg_->header);

  data_ready_ = true;

  // Don't publish a transform. robot_localization will fuse our estimates and do that
  mutex_exit(&lock_);
}

void PubTelemetry::publish() {
  mutex_enter_blocking(&lock_);
  if (data_ready_) {
    status_ = rcl_publish(&publisher_, msg_, NULL);
    data_ready_ = false;
  }

  mutex_exit(&lock_);
}
