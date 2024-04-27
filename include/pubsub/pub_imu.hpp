#ifndef PUB_IMU_HPP
#define PUB_IMU_HPP

#include "imu.hpp"
#include "pubsub/pubsub.hpp"
#include "constants.h"

/**
 * @class PubImu
 * @brief Class for publishing IMU data
 */
class PubImu {
 public:
  /**
   * @brief Get the status of the PubImu object
   * @return The status as an int16_t
   */
  int16_t get_status() {
    return status_;
  }

  /**
   * @brief Constructor for PubImu class
   * @param node Pointer to the rcl_node_t object
   * @param support Pointer to the rclc_support_t object
   * @param allocator Pointer to the rcl_allocator_t object
   * @param timer_hz The frequency of the timer in Hz (default: 10)
   * @param topic_name The name of the topic (default: "~/imu")
   * @param frame_id The frame ID of the IMU (default: IMU_FRAME)
   */
  PubImu(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
         int64_t timer_hz = 10,
         const char *topic_name = "~/imu",
         const char *frame_id = IMU_FRAME);

  /**
   * @brief Publishes the IMU data
   */
  void publish();

  /**
   * @brief Calculates the IMU data
   */
  void calculate();

  /**
   * @brief Assignment operator for PubImu class
   * @param other The PubImu object to assign from
   * @return Reference to the assigned PubImu object
   */
  PubImu &operator=(const PubImu &) = default;

  /**
   * @brief Destructor for PubImu class
   */
  ~PubImu();

 private:
  rcl_node_t *node_;  ///< Pointer to the rcl_node_t object
  rcl_allocator_t *allocator_;  ///< Pointer to the rcl_allocator_t object
  rclc_support_t *support_;  ///< Pointer to the rclc_support_t object
  rclc_executor_t *executor_;  ///< Pointer to the rclc_executor_t object

  sensor_msgs__msg__Imu *msg_;  ///< Pointer to the sensor_msgs__msg__Imu object

  rcl_publisher_t publisher_;  ///< The publisher object
  mutex_t lock_;  ///< The mutex object
  repeating_timer_t timer_;  ///< The repeating timer object
  IMU imu;  ///< The IMU object

  int16_t status_;  ///< The status of the PubImu object
  bool data_ready_;  ///< Flag indicating if data is ready

  /**
   * @brief Static trigger function for the repeating timer
   * @param rt Pointer to the repeating_timer_t object
   * @return True if the timer should continue, false otherwise
   */
  static bool trigger(repeating_timer_t *rt);
};

#endif  // PUB_IMU_HPP