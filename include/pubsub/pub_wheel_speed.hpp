#ifndef PUB_WHEELSPEED_HPP
#define PUB_WHEELSPEED_HPP

#include <rmw_microros/rmw_microros.h>

#include "constants.h"

#include "pubsub/pubsub.hpp"
#include "motor_manager.hpp"

/**
 * @class PubWheelSpeed
 * @brief Class for publishing wheel speed messages.
 */
class PubWheelSpeed {
 public:
  /**
   * @brief Get the status of the wheel speed publisher.
   * @return The status of the wheel speed publisher.
   */
  int16_t get_status() {
    return status_;
  }

  /**
   * @brief Constructor for PubWheelSpeed class.
   * @param node The ROS node.
   * @param support The ROS support object.
   * @param allocator The allocator for memory management.
   * @param motor_manager_ The motor manager object.
   * @param timer_hz The frequency of the timer in Hz (default: 10).
   * @param topic_name The name of the topic (default: "~/wheel_speed").
   * @param frame_id The frame ID (default: "base_link").
   */
  PubWheelSpeed(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
          MotorManager *motor_manager_,
          int64_t timer_hz = 10,
          const char *topic_name = "~/wheel_speed/out",
          const char *frame_id = "base_link");

  /**
   * @brief Publishes the wheel speed message.
   */
  void publish();

  /**
   * @brief Calculates the wheel speed.
   */
  void calculate();

  /**
   * @brief Destructor for PubWheelSpeed class.
   */
  ~PubWheelSpeed();

 private:
  rcl_node_t *node_; /**< The ROS node. */
  rcl_allocator_t *allocator_; /**< The allocator for memory management. */
  rclc_support_t *support_; /**< The ROS support object. */
  rclc_executor_t *executor_; /**< The ROS executor. */
  rcl_publisher_t publisher_; /**< The ROS publisher. */
  repeating_timer_t timer_; /**< The repeating timer. */
  mutex_t lock_; /**< The mutex lock. */

  sensor_msgs__msg__JointState* msg_; /**< The wheel speed message. */

  MotorManager *motor_manager_; /**< The motor manager object. */

  int16_t status_; /**< The status of the wheel speed publisher. */
  bool data_ready_; /**< Flag indicating if data is ready. */

  /**
   * @brief Static trigger function for the repeating timer.
   * @param rt The repeating timer.
   * @return True if the timer should continue, false otherwise.
   */
  static bool trigger(repeating_timer_t *rt);
};

#endif  // PUB_WHEELSPEED_HPP