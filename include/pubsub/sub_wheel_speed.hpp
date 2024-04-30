#ifndef SUB_WHEEL_SPEED_HPP
#define SUB_WHEEL_SPEED_HPP

#include "motor_manager.hpp"
#include "pubsub/pubsub.hpp"

/**
 * @class SubWheelSpeed
 * @brief Class for subscribing to and processing wheel_speed messages.
 */
class SubWheelSpeed {
 public:
  /**
   * @brief Get the status of the SubWheelSpeed object.
   * @return The status of the SubWheelSpeed object.
   */
  int16_t get_status() {
    return status_;
  }

  /**
   * @brief Constructor for SubWheelSpeed class.
   * @param node Pointer to the ROS node.
   * @param support Pointer to the ROS support structure.
   * @param allocator Pointer to the ROS allocator.
   * @param executor Pointer to the ROS executor.
   * @param motor_manager Pointer to the MotorManager object.
   * @param topic The topic to subscribe to (default: "~/wheel_speed").
   */
  SubWheelSpeed(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator, rclc_executor_t *executor,
            MotorManager *motor_manager, const char *topic = "~/wheel_speed/in");

  /**
   * @brief Subscribe to the wheel_speed topic.
   */
  void subscribe();

  /**
   * @brief Perform calculations based on received wheel_speed messages.
   */
  void calculate();

  /**
   * @brief Callback function for processing wheel_speed messages.
   * @param m Pointer to the received wheel_speed message.
   */
  void callback(const control_msgs__msg__MecanumDriveControllerState *m);

  /**
   * @brief Destructor for SubWheelSpeed class.
   */
  ~SubWheelSpeed();

 private:
  rcl_node_t *node_; /**< Pointer to the ROS node. */
  rcl_allocator_t *allocator_; /**< Pointer to the ROS allocator. */
  rclc_support_t *support_; /**< Pointer to the ROS support structure. */
  rclc_executor_t *executor_; /**< Pointer to the ROS executor. */
  rcl_subscription_t subscription_; /**< ROS subscription object. */
  const char *topic_; /**< The topic to subscribe to. */
  mutex_t lock_; /**< Mutex for thread safety. */
  control_msgs__msg__MecanumDriveControllerState *msg_; /**< Received wheel_speed message. */
  MotorManager *motor_manager_; /**< Pointer to the MotorManager object. */
  int16_t status_; /**< Status of the SubWheelSpeed object. */
  bool data_ready_; /**< Flag indicating if new data is ready. */

  /**
   * @brief Internal callback function for processing wheel_speed messages.
   * @param msgIn Pointer to the received wheel_speed message.
   */
  void internal_callback(const void *msgIn);
};


#endif  // SUB_WHEEL_SPEED_HPP