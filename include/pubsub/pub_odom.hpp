#ifndef PUB_ODOM_HPP
#define PUB_ODOM_HPP

#include "constants.h"

#include "Fusion.h"
#include "motor.hpp"
#include "motor_manager.hpp"
#include "pubsub/pubsub.hpp"

static double ceil_radians(double rad) {
  if (rad > M_PI) {
    return rad - 2 * M_PI;
  } else if (rad < -M_PI) {
    return rad + 2 * M_PI;
  } else {
    return rad;
  }
}

/**
 * @brief Class for publishing odometry data.
 */
class PubOdom {
 public:
  /**
   * @brief Get the status of the publisher.
   * @return The status of the publisher.
   */
  int16_t get_status() {
    return status_;
  }

  /**
   * @brief Constructor for PubOdom class.
   * @param node The ROS2 node.
   * @param support The ROS2 support object.
   * @param allocator The ROS2 allocator.
   * @param motor_manager_ The motor manager object.
   * @param timer_hz The frequency of the timer.
   * @param topic_name The name of the topic to publish to.
   * @param frame_id The frame ID for the odometry message.
   * @param child_frame_id The child frame ID for the odometry message.
   */
  PubOdom(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
          MotorManager *motor_manager_,
          int64_t timer_hz = 10,
          const char *topic_name = "~/odom",
          const char *frame_id = "odom",
          const char *child_frame_id = "base_link");

  /**
   * @brief Publishes the odometry message.
   */
  void publish();

  /**
   * @brief Calculates the odometry data.
   */
  void calculate();

  /**
   * @brief Static function to calculate quaternion from yaw angle.
   * @param yaw The yaw angle in radians.
   * @return The fusion quaternion.
   */
  static const FusionQuaternion quaternion_from_yaw(const Radians yaw);

  /**
   * @brief Destructor for PubOdom class.
   */
  ~PubOdom();

 private:
  rcl_node_t *node_;  // The ROS2 node.
  rcl_allocator_t *allocator_;  // The ROS2 allocator.
  rclc_support_t *support_;  // The ROS2 support object.
  rclc_executor_t *executor_;  // The ROS2 executor.
  rcl_publisher_t publisher_;  // The ROS2 publisher.
  mutex_t lock_;  // The mutex lock.
  repeating_timer_t timer_;  // The repeating timer.

  // Odometry
  Radians yaw_ = 0;  // The yaw angle.
  Micrometers x_ = 0;  // The x position.
  Micrometers y_ = 0;  // The y position.
  nav_msgs__msg__Odometry *msg_;  // The odometry message.

  MotorManager *motor_manager_;  // The motor manager object.

  int16_t status_;  // The status of the publisher.
  bool data_ready_;  // Flag to indicate if data is ready.

  /**
   * @brief Static function to trigger the repeating timer.
   * @param rt The repeating timer object.
   * @return True if the timer should continue, false otherwise.
   */
  static bool trigger(repeating_timer_t *rt);
};

#endif  // PUB_ODOM_HPP