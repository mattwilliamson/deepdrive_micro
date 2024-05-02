#ifndef PUB_SONAR_HPP
#define PUB_SONAR_HPP

#include <rmw_microros/rmw_microros.h>

#include "HCSR04.cpp"
#include "constants.h"
#include "motor_manager.hpp"
#include "pubsub/pubsub.hpp"

extern "C" {
#include "config.h"
}

/**
 * @class PubSonar
 * @brief Class for publishing sonar messages.
 */
class PubSonar {
 public:
  /**
   * @brief Get the status of the sonar publisher.
   * @return The status of the sonar publisher.
   */
  int16_t get_status() {
    return status_;
  }

  /**
   * @brief PubSonar class for publishing HC-SR04 sonar data.
   *
   * This class provides functionality to publish sonar data from an HC-SR04 sensor.
   * It initializes the necessary parameters and sets up a timer to periodically publish the data.
   *
   * @param node A pointer to the ROS2 node.
   * @param support A pointer to the ROS2 support structure.
   * @param allocator A pointer to the ROS2 allocator.
   * @param timer_hz The frequency at which the timer should trigger the publishing of sonar data. Default is 10 Hz.
   * @param trigger_pin The pin number for the trigger pin of the HC-SR04 sensor.
   * @param echo_pin The pin number for the echo pin of the HC-SR04 sensor.
   * @param topic_name The name of the topic to publish the sonar data. Default is "~/sonar/out".
   * @param frame_id The frame ID for the sonar data. Default is "base_link".
   */
  PubSonar(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
           uint trigger_pin,
           uint echo_pin,
           int64_t timer_hz = 10,
           const char *topic_name = "~/sonar/out",
           const char *frame_id = "base_link");

  /**
   * @brief Publishes the sonar message.
   */
  void publish();

  /**
   * @brief Calculates the sonar.
   */
  void calculate();

  /**
   * @brief Destructor for PubSonar class.
   */
  ~PubSonar();

 private:
  rcl_node_t *node_;             /**< The ROS node. */
  rcl_allocator_t *allocator_;   /**< The allocator for memory management. */
  rclc_support_t *support_;      /**< The ROS support object. */
  rclc_executor_t *executor_;    /**< The ROS executor. */
  rcl_publisher_t publisher_;    /**< The ROS publisher. */
  repeating_timer_t timer_;      /**< The repeating timer. */
  mutex_t lock_;                 /**< The mutex lock. */
  sensor_msgs__msg__Range *msg_; /**< The sonar message. */

  uint pin_trigger_; /**< The trigger pin for the sonar sensor. */
  uint pin_echo_;    /**< The echo pin for the sonar sensor. */

  HCSR04 *sonar_; /**< The sonar sensor object. */

  int16_t status_;  /**< The status of the sonar publisher. */
  bool data_ready_; /**< Flag indicating if data is ready. */

  // Really for debugging
  double samples_[SONAR_SAMPLES]; /**< The buffer to store the sonar samples. */
  uint sample_index_;             /**< The index of the current sample. */

  /**
   * @brief Static trigger function for the repeating timer.
   * @param rt The repeating timer.
   * @return True if the timer should continue, false otherwise.
   */
  static bool trigger(repeating_timer_t *rt);
};

#endif  // PUB_SONAR_HPP