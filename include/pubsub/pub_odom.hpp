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

class PubOdom {
 public:
  int16_t get_status() {
    return status_;
  }

  PubOdom(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
          MotorManager *motor_manager_,
          int64_t timer_hz = 10,
          const char *topic_name = "~/odom",
          const char *frame_id = "odom",
          const char *child_frame_id = "base_link");

  void publish();
  void calculate();

  static const FusionQuaternion quaternion_from_yaw(const Radians yaw);

  ~PubOdom();

 private:
  rcl_node_t *node_;
  rcl_allocator_t *allocator_;
  rclc_support_t *support_;
  rclc_executor_t *executor_;
  rcl_publisher_t publisher_;
  mutex_t lock_;
  repeating_timer_t timer_;

  // Odometry
  Radians yaw_ = 0;
  Micrometers x_ = 0;
  Micrometers y_ = 0;
  nav_msgs__msg__Odometry *msg_;

  MotorManager *motor_manager_;

  int16_t status_;
  bool data_ready_;

  static bool trigger(repeating_timer_t *rt);
};

#endif  // PUB_ODOM_HPP