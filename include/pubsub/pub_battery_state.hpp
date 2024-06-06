#ifndef PUB_BATTERY_STATE_HPP
#define PUB_BATTERY_STATE_HPP

#include <limits>

#include "analog_sensors.hpp"
#include "pubsub/pubsub.hpp"
#include "buzzer.hpp"
#include "status.hpp"
#include "constants.h"

/**
 * @class PubBatteryState
 * @brief Class for publishing battery state information.
 */
class PubBatteryState {
 public:
  /**
   * @brief Get the battery status.
   * @return The battery status as an int16_t.
   */
  int16_t get_status() {
    return status_;
  }

  /**
   * @brief Constructor for PubBatteryState.
   * @param node The ROS2 node.
   * @param support The ROS2 support object.
   * @param allocator The ROS2 allocator.
   * @param analog_sensors Pointer to the AnalogSensors object.
   * @param timer_hz The frequency of the battery state loop in Hz.
   * @param topic_name The name of the battery topic.
   * @param frame_id The frame ID for the battery state.
   */
  PubBatteryState(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
              AnalogSensors *analog_sensors, 
              Buzzer *buzzer,
               int64_t timer_hz = BATTERY_STATE_LOOP_HZ,
               const char *topic_name = "~/battery",
               const char *frame_id = BATTERY_FRAME);

  /**
   * @brief Publish the battery state message.
   */
  void publish();

  /**
   * @brief Calculate the battery state.
   */
  void calculate();

  /**
   * @brief Play a buzzer if hte battery is low and such.
   */
  void checkBuzzer();

  /**
   * @brief Destructor for PubBatteryState.
   */
  ~PubBatteryState();

 private:
  rcl_node_t *node_; /**< The ROS2 node. */
  rcl_allocator_t *allocator_; /**< The ROS2 allocator. */
  rclc_support_t *support_; /**< The ROS2 support object. */
  rclc_executor_t *executor_; /**< The ROS2 executor. */
  rcl_publisher_t publisher_; /**< The ROS2 publisher. */
  mutex_t lock_; /**< The mutex lock. */
  repeating_timer_t timer_; /**< The repeating timer. */

  sensor_msgs__msg__BatteryState *msg_; /**< The battery state message. */

  AnalogSensors *analog_sensors_; /**< Pointer to the AnalogSensors object. */
  Buzzer *buzzer_; /**< Pointer to the Buzzer object. */

  int16_t status_; /**< The battery status. */
  bool data_ready_; /**< Flag indicating if data is ready. */

  int64_t buzzer_last_played_ = 0; /**< The last time the buzzer was played. */

  /**
   * @brief Static trigger function for the repeating timer.
   * @param rt Pointer to the repeating timer.
   * @return True if the timer should continue, false otherwise.
   */
  static bool trigger(repeating_timer_t *rt);
};

#endif  // PUB_BATTERY_STATE_HPP