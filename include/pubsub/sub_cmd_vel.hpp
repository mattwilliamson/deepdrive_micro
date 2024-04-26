#ifndef SUB_CMD_VEL_HPP
#define SUB_CMD_VEL_HPP

#include "motor_manager.hpp"
#include "pubsub/pubsub.hpp"

class SubCmdVel {
 public:
  int16_t get_status() {
    return status_;
  }

  SubCmdVel(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator, rclc_executor_t *executor,
            MotorManager *motor_manager, const char *topic = "~/cmd_vel");

  void subscribe();
  void calculate();
  void callback(const geometry_msgs__msg__Twist *m);

  ~SubCmdVel();

 private:
  

  rcl_node_t *node_;
  rcl_allocator_t *allocator_;
  rclc_support_t *support_;
  rclc_executor_t *executor_;
  rcl_subscription_t subscription_;
  const char *topic_;
  mutex_t lock_;

  geometry_msgs__msg__Twist msg_;

  MotorManager *motor_manager_;

  int16_t status_;
  bool data_ready_;

  void internal_callback(const void *msgIn);
};

#endif  // SUB_CMD_VEL_HPP