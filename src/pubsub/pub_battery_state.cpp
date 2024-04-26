#include "pubsub/pub_battery_state.hpp"

static bool _pub_battery_state_triggered;

bool PubBatteryState::trigger(repeating_timer_t *rt) {
  _pub_battery_state_triggered = true;
  return true;
}

PubBatteryState::PubBatteryState(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
                                 AnalogSensors *analog_sensors,
                                 int64_t timer_hz,
                                 const char *topic_name) {
  node_ = node;
  allocator_ = allocator;
  support_ = support;
  analog_sensors_ = analog_sensors;
  msg_ = sensor_msgs__msg__BatteryState__create();

  msg_->location = micro_ros_string_utilities_init("base_link");
  msg_->serial_number = micro_ros_string_utilities_init("1234567890");
  msg_->design_capacity = BATTERY_CAPACITY;
  msg_->power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;

  mutex_init(&lock_);

  data_ready_ = false;

  status_ = rclc_publisher_init_default(
      &publisher_, node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      topic_name);

  if (!add_repeating_timer_us(-MICROSECONDS / timer_hz, PubBatteryState::trigger, NULL, &timer_)) {
    printf("Failed to add control loop timer\r\n");
    status_ = -1;
  }
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

  mutex_exit(&lock_);
}

PubBatteryState::~PubBatteryState() {
  cancel_repeating_timer(&timer_);
  status_ = rcl_publisher_fini(&publisher_, node_);
}