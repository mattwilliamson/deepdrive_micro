#include "pubsub/pub_battery_state.hpp"

static bool _pub_battery_state_triggered;

bool PubBatteryState::trigger(repeating_timer_t *rt) {
  _pub_battery_state_triggered = true;
  return true;
}

PubBatteryState::PubBatteryState(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
                                 AnalogSensors *analog_sensors,
                                 Buzzer *buzzer,
                                 int64_t timer_hz,
                                 const char *topic_name,
                                 const char *frame_id) {
  node_ = node;
  allocator_ = allocator;
  support_ = support;
  analog_sensors_ = analog_sensors;
  buzzer_ = buzzer;
  buzzer_last_played_ = 0;

  msg_ = sensor_msgs__msg__BatteryState__create();

  msg_->header.frame_id = micro_ros_string_utilities_init(frame_id);
  msg_->location = micro_ros_string_utilities_init("bottom");
  msg_->serial_number = micro_ros_string_utilities_init("DEADBEEF");
  msg_->design_capacity = BATTERY_CAPACITY;
  msg_->power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;

  mutex_init(&lock_);

  data_ready_ = false;

  status_ = rclc_publisher_init_default(
      &publisher_, node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      topic_name);

  assert(add_repeating_timer_us(-MICROSECONDS / timer_hz, PubBatteryState::trigger, NULL, &timer_));
}


void PubBatteryState::calculate() {
  if (!_pub_battery_state_triggered) {
    return;
  }

  mutex_enter_blocking(&lock_);

  msg_->voltage = analog_sensors_->getBatteryVoltage();
  msg_->percentage = AnalogSensors::convertVoltageToPercentage(msg_->voltage);

  msg_->present = true;
  msg_->temperature = analog_sensors_->getTemperature();

  if (msg_->voltage > 2.0) {
    msg_->power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    msg_->present = true;
  } else {
    msg_->power_supply_status =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
    msg_->present = false;
  }

  // Set timestamp
  PubSub::set_timestamp_header(&msg_->header);

  data_ready_ = true;

  // Don't publish a transform. robot_localization will fuse our estimates and do that
  mutex_exit(&lock_);
}

void PubBatteryState::publish() {
  mutex_enter_blocking(&lock_);
  if (data_ready_) {
    status_ = rcl_publish(&publisher_, msg_, NULL);
    data_ready_ = false;
  }

  checkBuzzer();

  mutex_exit(&lock_);
}

void PubBatteryState::checkBuzzer() {
  // TODO: Make this a bit more generic. maybe an event system?
  
  if (msg_->percentage < BUZZER_BATTERY_ERROR) {
    if (rmw_uros_epoch_nanos() - buzzer_last_played_ > BUZZER_WARN_INTERVAL) {
      buzzer_->playTune(Buzzer::Tune::ERROR);
      buzzer_last_played_ = rmw_uros_epoch_nanos();
      StatusManager::getInstance().set(Status::Error);
    }

  } else if (msg_->percentage < BUZZER_BATTERY_WARN) {
    if (rmw_uros_epoch_nanos() - buzzer_last_played_ > BUZZER_WARN_INTERVAL) {
      buzzer_->playTune(Buzzer::Tune::WARNING);
      buzzer_last_played_ = rmw_uros_epoch_nanos();
      StatusManager::getInstance().set(Status::Warning);
    }
  } else {
    buzzer_->stop();
    // StatusManager::getInstance().set(Status::Active);
  }
}

PubBatteryState::~PubBatteryState() {
  cancel_repeating_timer(&timer_);
  status_ = rcl_publisher_fini(&publisher_, node_);
}