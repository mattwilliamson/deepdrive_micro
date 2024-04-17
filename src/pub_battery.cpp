#include "node.hpp"

int Node::init_battery() {
  RCCHECK(rclc_publisher_init_default(
      &publisher_battery, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      "~/battery"));

  msg_out_battery.location = micro_ros_string_utilities_init("base_link");
  msg_out_battery.serial_number = micro_ros_string_utilities_init("1234567890");
  msg_out_battery.design_capacity = BATTERY_CAPACITY;
  msg_out_battery.power_supply_status =
      sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;

  return 0;
}

void Node::publish_battery() {
  // TODO: Do this in the other core
  msg_out_battery.voltage = analog_sensors->getBatteryVoltage();
  msg_out_battery.percentage =
      AnalogSensors::convertVoltageToPercentage(msg_out_battery.voltage);

  msg_out_battery.present = true;
  msg_out_battery.temperature = analog_sensors->getTemperature();

  if (msg_out_battery.voltage > 2.0) {
    msg_out_battery.power_supply_status =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    msg_out_battery.present = true;
  } else {
    msg_out_battery.power_supply_status =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
    msg_out_battery.present = false;
  }

  RCSOFTCHECK(rcl_publish(&publisher_battery, &msg_out_battery, NULL));
}
