#ifndef SUB_CMD_VEL_HPP
#define SUB_CMD_VEL_HPP

#include "motor_manager.hpp"
#include "pubsub/pubsub.hpp"

/**
 * @class SubCmdVel
 * @brief Class for subscribing to and processing cmd_vel messages.
 */
class SubCmdVel {
 public:
  /**
   * @brief Get the status of the SubCmdVel object.
   * @return The status of the SubCmdVel object.
   */
  int16_t get_status() {
    return status_;
  }

  /**
   * @brief Constructor for SubCmdVel class.
   * @param node Pointer to the ROS node.
   * @param support Pointer to the ROS support structure.
   * @param allocator Pointer to the ROS allocator.
   * @param executor Pointer to the ROS executor.
   * @param motor_manager Pointer to the MotorManager object.
   * @param topic The topic to subscribe to (default: "~/cmd_vel").
   */
  SubCmdVel(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator, rclc_executor_t *executor,
            MotorManager *motor_manager, const char *topic = "/cmd_vel");

  /**
   * @brief Subscribe to the cmd_vel topic.
   */
  void subscribe();

  /**
   * @brief Perform calculations based on received cmd_vel messages.
   */
  void calculate();

  /**
   * @brief Callback function for processing cmd_vel messages.
   * @param m Pointer to the received cmd_vel message.
   */
  void callback(const geometry_msgs__msg__Twist *m);

  /**
   * @brief Get the timestamp of the last received message.
   * @return The timestamp of the last received message.
   */
  int64_t getLastMessage() {
    return last_message_time_;
  }
  
  /**
   * @brief Reset the timestamp of the last received message.
   */
  void resetLastMessage() {
    last_message_time_ = 0;
  }

  /**
   * @brief Destructor for SubCmdVel class.
   */
  ~SubCmdVel();

 private:
  rcl_node_t *node_; /**< Pointer to the ROS node. */
  rcl_allocator_t *allocator_; /**< Pointer to the ROS allocator. */
  rclc_support_t *support_; /**< Pointer to the ROS support structure. */
  rclc_executor_t *executor_; /**< Pointer to the ROS executor. */
  rcl_subscription_t subscription_; /**< ROS subscription object. */
  const char *topic_; /**< The topic to subscribe to. */
  mutex_t lock_; /**< Mutex for thread safety. */
  MotorManager *motor_manager_; /**< Pointer to the MotorManager object. */
  int16_t status_; /**< Status of the SubCmdVel object. */
  bool data_ready_; /**< Flag indicating if new data is ready. */
  geometry_msgs__msg__Twist *msg_; /**< Received cmd_vel message. */
  geometry_msgs__msg__Twist *prev_msg_; /**< Received cmd_vel message. */
  int64_t last_message_time_ = 0; /**< Timestamp of the last received message. */
  

  /**
   * @brief Internal callback function for processing cmd_vel messages.
   * @param msgIn Pointer to the received cmd_vel message.
   */
  void internal_callback(const void *msgIn);
};


#endif  // SUB_CMD_VEL_HPP