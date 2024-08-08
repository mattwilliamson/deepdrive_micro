#ifndef PUBSUB_HPP
#define PUBSUB_HPP

extern "C" {
#include <rclc/executor.h>
// #include <rcl/rcl.h>
// #include <control_msgs/msg/mecanum_drive_controller_state.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_utilities/string_utilities.h>
#include <nav_msgs/msg/odometry.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/sequence_bound.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
}

#include "config.h"

/**
 * @class PubSub
 * @brief A class that provides functionality for publishing and subscribing to messages.
 */
class PubSub {
 public:
  /**
   * @brief Sets the timestamp header of a message to the current time.
   * @param header A pointer to the message header.
   */
  static void set_timestamp_header(std_msgs__msg__Header *header) {
    int64_t t = rmw_uros_epoch_nanos();
    header->stamp.sec = RCUTILS_NS_TO_S(t);
    header->stamp.nanosec = t % 1000000000;
  }

  /**
   * @brief Constructs a PubSub object.
   * @param node A pointer to the ROS node.
   * @param allocator A pointer to the allocator.
   */
  PubSub(rcl_node_t *node, rcl_allocator_t *allocator) {
    node_ = node;
    allocator_ = allocator;
  }

  /**
   * @brief Destroys the PubSub object.
   */
  ~PubSub() {
    if (publisher_ != nullptr) {
      rcl_publisher_fini(publisher_, node_);
    }
  }

 private:
  rcl_node_t *node_;  ///< A pointer to the ROS node.
  rcl_allocator_t *allocator_;  ///< A pointer to the allocator.
  rclc_support_t *support_;  ///< A pointer to the RCLC support structure.
  rclc_executor_t *executor_;  ///< A pointer to the RCLC executor structure.
  rcl_publisher_t *publisher_;  ///< A pointer to the ROS publisher.
  mutex_t lock_;  ///< A mutex for thread safety.
  repeating_timer_t timer_;  ///< A repeating timer for periodic tasks.
};

#endif  // PUBSUB_HPP