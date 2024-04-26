#ifndef PUB_TELEMETRY_HPP
#define PUB_TELEMETRY_HPP

#include "constants.h"
#include "motor_manager.hpp"
#include "pubsub/pubsub.hpp"
#include "status.hpp"

namespace Diagnostics {
const int IMU = 0;
const int CORE_0 = 1;
const int CORE_1 = 2;
}  // namespace Diagnostics

/**
 * @class PubTelemetry
 * @brief Class for publishing telemetry data.
 */
class PubTelemetry {
 public:
  /**
   * @brief Get the status value.
   * @return The status value.
   */
  int16_t get_status() {
    return status_;
  }

  /**
   * @brief Constructor for PubTelemetry class.
   * @param node The ROS2 node.
   * @param support The ROS2 support object.
   * @param allocator The ROS2 allocator.
   * @param motor_manager The motor manager object.
   * @param timer_hz The frequency of the telemetry loop.
   * @param topic_name The name of the telemetry topic.
   */
  PubTelemetry(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
               MotorManager *motor_manager,
               int64_t timer_hz = TELEMETRY_LOOP_HZ,
               const char *topic_name = "/diagnostics");

  /**
   * @brief Publishes the telemetry data.
   */
  void publish();

  /**
   * @brief Calculates the telemetry data.
   */
  void calculate();

  /**
   * @brief Sets the start time for the core for telemetry calculation.
   * @param core The core number.
   */
  void set_core_start(int core);

  /**
   * @brief Sets the stop time for telemetry calculation.
   * @param core The core number.
   * @return The elapsed time.
   */
  uint64_t set_core_stop(int core);

  /**
   * @brief Gets the elapsed time for the core for telemetry calculation.
   * @param core The core number.
   * @return The elapsed time.
   */
  uint64_t get_core_elapsed(int core);

  /**
   * @brief Destructor for PubTelemetry class.
   */
  ~PubTelemetry();

 private:
  rcl_node_t *node_;                            ///< The ROS2 node.
  rcl_allocator_t *allocator_;                  ///< The ROS2 allocator.
  rclc_support_t *support_;                     ///< The ROS2 support object.
  rclc_executor_t *executor_;                   ///< The ROS2 executor.
  rcl_publisher_t publisher_;                   ///< The ROS2 publisher.
  mutex_t lock_;                                ///< The mutex lock.
  repeating_timer_t timer_;                     ///< The repeating timer.
  diagnostic_msgs__msg__DiagnosticArray *msg_;  ///< The diagnostic message.

  MotorManager *motor_manager_;  ///< The motor manager object.
  int16_t status_;               ///< The status value.
  bool data_ready_;              ///< Flag indicating if data is ready.

  uint64_t core_start_[2];    ///< Array to store start times for telemetry calculation.
  uint64_t core_elapsed_[2];  ///< Array to store elapsed times for telemetry calculation.

  /**
   * @brief Static function to trigger the repeating timer.
   * @param rt The repeating timer object.
   * @return True if successful, false otherwise.
   */
  static bool trigger(repeating_timer_t *rt);
};

#endif  // PUB_TELEMETRY_HPP