#include "pubsub/pubsub.hpp"

// Statically allocate memory
// https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/
// https://github.com/micro-ROS/micro-ROS-demos/blob/humble/rclc/static_type_handling/main.c
// mypackage__msg__MyType mymsg;

// static micro_ros_utilities_memory_conf_t conf = {0};

// conf.max_string_capacity = 50;
// conf.max_ros2_type_sequence_capacity = 5;
// conf.max_basic_type_sequence_capacity = 5;

// bool success = micro_ros_utilities_create_message_memory(
//   ROSIDL_GET_MSG_TYPE_SUPPORT(mypackage, msg, MyType),
//   &mymsg,
//   conf
// );