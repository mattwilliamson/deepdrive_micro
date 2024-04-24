#include "node.hpp"

// #include <rclc/rclc.h>

// extern "C" {
//   #include <diagnostic_msgs/msg/diagnostic_status.h>
//   #include <diagnostic_msgs/msg/diagnostic_array.h>
//   #include <diagnostic_msgs/msg/key_value.h>

//   #include "config.h"
// }

#include "pub_diagnostic.hpp"
// #include "led_ring.hpp"
// #include "status.hpp"

int Node::init_diagnostic() {
  RCCHECK(rclc_publisher_init_default(
      &publisher_diagnostic, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
      "/diagnostics"));

  msg_out_diagnostic.header.frame_id = micro_ros_string_utilities_init(DIAGNOSTIC_FRAME);
  diagnostic_msgs__msg__DiagnosticStatus__Sequence__init(&msg_out_diagnostic.status, DIAGNOSTIC_COUNT);

  msg_out_diagnostic.status.data[0].hardware_id = micro_ros_string_utilities_init("deepdrive_micro");
  msg_out_diagnostic.status.data[0].name = micro_ros_string_utilities_init("CPU Loop Time (Hz)");

  diagnostic_msgs__msg__KeyValue__Sequence__init(&msg_out_diagnostic.status.data[0].values, DIAGNOSTIC_ROWS);
  msg_out_diagnostic.status.data[0].values.data[0].key = micro_ros_string_utilities_init("Core 0");
  msg_out_diagnostic.status.data[0].values.data[1].key = micro_ros_string_utilities_init("Core 1");
  msg_out_diagnostic.status.data[0].values.data[2].key = micro_ros_string_utilities_init("IMU");
  msg_out_diagnostic.status.data[0].values.data[3].key = micro_ros_string_utilities_init("Error");


  return 0;
}

void Node::publish_diagnostic() {
  PubSub::set_timestamp_header(&msg_out_diagnostic.header);

  // Convert core_elapsed[0] to string
  std::string core_elapsed_0_str = std::to_string((double)MICROSECONDS / core_elapsed[0]);
  msg_out_diagnostic.status.data[0].values.data[0].value.data = const_cast<char*>(core_elapsed_0_str.c_str());

  // Convert core_elapsed[1] to string
  std::string core_elapsed_1_str = std::to_string((double)MICROSECONDS / core_elapsed[1]);
  msg_out_diagnostic.status.data[0].values.data[1].value.data = const_cast<char*>(core_elapsed_1_str.c_str());

  // IMU Status
  char *imu_status = const_cast<char*>(imu.statusString());
  msg_out_diagnostic.status.data[0].values.data[2].value.data = imu_status;

  // Error message
  std::string error_message = status.getErrorString();
  msg_out_diagnostic.status.data[0].values.data[3].value.data = const_cast<char*>(error_message.c_str());

  // TODO: set error string from status
  msg_out_diagnostic.status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__OK;
  msg_out_diagnostic.status.data[0].message = micro_ros_string_utilities_init("OK");

  // TODO: Set status string
  switch (status.get()) {
    case Status::Connecting:
      msg_out_diagnostic.status.data[0].level =
          diagnostic_msgs__msg__DiagnosticStatus__WARN;
      break;
    case Status::Connected:
      msg_out_diagnostic.status.data[0].level =
          diagnostic_msgs__msg__DiagnosticStatus__OK;
      break;
    case Status::Active:
      msg_out_diagnostic.status.data[0].level =
          diagnostic_msgs__msg__DiagnosticStatus__OK;
      break;
    case Status::Error:
      msg_out_diagnostic.status.data[0].level =
          diagnostic_msgs__msg__DiagnosticStatus__ERROR;
      msg_out_diagnostic.status.data[0].message =
          micro_ros_string_utilities_init("Error");
      break;
    case Status::Rebooted:
      msg_out_diagnostic.status.data[0].level =
          diagnostic_msgs__msg__DiagnosticStatus__ERROR;
      break;
  }

  // TODO: Send Status
  if (!rmw_uros_epoch_synchronized()) {
    msg_out_diagnostic.status.data[0].message =
        micro_ros_string_utilities_init("Time not synchronized");
  }

  RCSOFTCHECK(rcl_publish(&publisher_diagnostic, &msg_out_diagnostic, NULL));
}