#include "pubsub/pub_telemetry.hpp"

static bool _pub_telemetry_triggered;

bool PubTelemetry::trigger(repeating_timer_t *rt) {
  _pub_telemetry_triggered = true;
  return true;
}

void PubTelemetry::set_core_start(int core) {
  core_start_[core] = time_us_64();
}

uint64_t PubTelemetry::set_core_stop(int core) {
  core_elapsed_[core] = time_us_64() - core_start_[core];
  return get_core_elapsed(core);
}

uint64_t PubTelemetry::get_core_elapsed(int core) {
  return core_elapsed_[core];
}

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
  diagnostic_msgs__msg__KeyValue__Sequence__init(&msg_->status.data[0].values, Diag::TOTAL);

  diagnostic_msgs__msg__DiagnosticStatus *diag = &msg_->status.data[0];

  diag->hardware_id = micro_ros_string_utilities_init("deepdrive_micro");
  diag->name = micro_ros_string_utilities_init("DeepDrive Status");
  diag->message = micro_ros_string_utilities_init_with_size(DIAGNOSTIC_MESSAGE_LEN);

  diag->values.data[Diag::IMU].key = micro_ros_string_utilities_init("IMU Status");

  // Preallocate these strings
  // diag->values.data[Diag::CORE_0].key = micro_ros_string_utilities_init("Core 0 Loop Time (us)");
  diag->values.data[Diag::CORE_0].key = micro_ros_string_utilities_init("Core 0 Loop Rate (Hz)");
  diag->values.data[Diag::CORE_0].value = micro_ros_string_utilities_init_with_size(DIAGNOSTIC_NUMBER_LEN);

  // diag->values.data[Diag::CORE_1].key = micro_ros_string_utilities_init("Core 1 Loop Time (us)");
  diag->values.data[Diag::CORE_1].key = micro_ros_string_utilities_init("Core 1 Loop Rate (Hz)");
  diag->values.data[Diag::CORE_1].value = micro_ros_string_utilities_init_with_size(DIAGNOSTIC_NUMBER_LEN);

  diag->values.data[Diag::MEM_FREE].key = micro_ros_string_utilities_init("Memory Free (bytes)");
  diag->values.data[Diag::MEM_FREE].value = micro_ros_string_utilities_init_with_size(DIAGNOSTIC_NUMBER_LEN);

  diag->values.data[Diag::MEM_USED].key = micro_ros_string_utilities_init("Memory Used (bytes)");
  diag->values.data[Diag::MEM_USED].value = micro_ros_string_utilities_init_with_size(DIAGNOSTIC_NUMBER_LEN);

  assert(add_repeating_timer_us(-MICROSECONDS / timer_hz, PubTelemetry::trigger, NULL, &timer_));
}

void PubTelemetry::calculate() {
  if (!_pub_telemetry_triggered) {
    return;
  }

  mutex_enter_blocking(&lock_);

  //   auto motors = motor_manager_->get_motors();

  // this ping is taking too long
  // const int timeout_ms = 1000;
  // const int attempts = 1;

  // rmw_ret_t error_code = rmw_uros_ping_agent(timeout_ms, attempts);

  // TODO: Probably want to check other criteria when disabling motors
  // TODO: Check if two motors on the same side have similar speeds
  // TODO: Check fore multiple ping failures with shorter timeout
  // TODO: This was failing on and off
  // if (error_code != RMW_RET_OK) {
  //   StatusManager::getInstance().set(Status::Error);
  //   motor_manager_->disable_motors();
  // } else {
  // StatusManager::getInstance().set(Status::Error);
  // motor_manager_->enable_motors();
  // }

  diagnostic_msgs__msg__DiagnosticStatus *diag = &msg_->status.data[0];

  // itoa(core_elapsed_[0], diag->values.data[Diag::CORE_0].value.data, 10);
  // itoa(core_elapsed_[1], diag->values.data[Diag::CORE_1].value.data, 10);
  // Hz
  // itoa(MICROSECONDS / core_elapsed_[0], diag->values.data[Diag::CORE_0].value.data, 10);
  // itoa(MICROSECONDS / core_elapsed_[1], diag->values.data[Diag::CORE_1].value.data, 10);

  snprintf(diag->values.data[Diag::CORE_0].value.data, DIAGNOSTIC_NUMBER_LEN, "%f", (double)MICROSECONDS / core_elapsed_[0]);
  snprintf(diag->values.data[Diag::CORE_1].value.data, DIAGNOSTIC_NUMBER_LEN, "%f", (double)MICROSECONDS / core_elapsed_[1]);

  itoa(get_free_heap(), diag->values.data[Diag::MEM_FREE].value.data, 10);
  itoa(get_used_heap(), diag->values.data[Diag::MEM_USED].value.data, 10);

  // Update status from status manager
  StatusManager &status_manager = StatusManager::getInstance();

  if (!rmw_uros_epoch_synchronized()) {
    status_manager.set(Status::TimeNotSynchronized);
    status_manager.setErrorString("Time not synchronized");
  } else {
    status_manager.removeStatus(Status::TimeNotSynchronized);
  }

  diag->level = diagnostic_msgs__msg__DiagnosticStatus__OK;
  setDiagMessage(diag, "OK");

  Status status = status_manager.get();
  std::string status_message = status_manager.getErrorString();

  // TODO: Set status string
  diag->message.data = status_message.data();

  switch (status) {
    case Status::Connecting:
      diag->level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
      break;
    case Status::Connected:
      diag->level = diagnostic_msgs__msg__DiagnosticStatus__OK;
      break;
    case Status::Active:
      diag->level = diagnostic_msgs__msg__DiagnosticStatus__OK;
      break;
    case Status::Error:
      diag->level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
      setDiagMessage(diag, status_message.data());
      break;
    case Status::Rebooted:
      diag->level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
      break;
  }

  // Set timestamp
  PubSub::set_timestamp_header(&msg_->header);

  data_ready_ = true;

  // Don't publish a transform. robot_localization will fuse our estimates and do that
  mutex_exit(&lock_);
}

void PubTelemetry::setDiagMessage(diagnostic_msgs__msg__DiagnosticStatus *diag, const char* status_message) {
  // static const std::string padding(DIAGNOSTIC_MESSAGE_LEN, ' ');
  // strncpy(diag->message.data, padding.data(), DIAGNOSTIC_MESSAGE_LEN);
  // strncpy(diag->message.data, status_message, strlen(status_message));
  // for (int i = 0; i < DIAGNOSTIC_MESSAGE_LEN; i++) {
  //   diag->message.data[i] = ' ';
  // }
  // diag->message.size = strlen(status_message);
  // strncpy(diag->message.data, status_message, diag->message.size);
}

void PubTelemetry::publish() {
  mutex_enter_blocking(&lock_);
  if (data_ready_) {
    status_ = rcl_publish(&publisher_, msg_, NULL);
    data_ready_ = false;
  }

  mutex_exit(&lock_);
}

PubTelemetry::~PubTelemetry() {
  cancel_repeating_timer(&timer_);
  status_ = rcl_publisher_fini(&publisher_, node_);
}