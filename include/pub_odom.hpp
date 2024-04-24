#ifndef PUB_ODOM_HPP
#define PUB_ODOM_HPP

#include <algorithm>
#include <cmath>
#include <vector>

extern "C" {
#include <micro_ros_utilities/string_utilities.h>
#include <nav_msgs/msg/odometry.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "constants.h"
}

#include "motor.hpp"
#include "pubsub.hpp"
#include "quaternion.hpp"

static double ceil_radians(double rad) {
  if (rad > M_PI) {
    return rad - 2 * M_PI;
  } else if (rad < -M_PI) {
    return rad + 2 * M_PI;
  } else {
    return rad;
  }
}

static bool _pub_odom_triggered;

class PubOdom {
 public:
  int16_t get_status() {
    return status_;
  }

  PubOdom(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
          std::vector<Motor *> motors,
          int64_t timer_hz = 10,
          const char *topic_name = "~/odom",
          const char *frame_id = "odom",
          const char *child_frame_id = "base_link");

  void publish();
  void calculate();

  ~PubOdom() {
    // if (publisher_ != nullptr) {
      rcl_publisher_fini(&publisher_, node_);
    // }
  }

 private:
  rcl_node_t *node_;
  rcl_allocator_t *allocator_;
  rclc_support_t *support_;
  rclc_executor_t *executor_;

  // Odometry
  Radians yaw_ = 0;
  Micrometers x_ = 0;
  Micrometers y_ = 0;
  nav_msgs__msg__Odometry *msg_;

  rcl_publisher_t publisher_;
  mutex_t lock_;
  repeating_timer_t timer_;
  std::vector<Motor *> motors_;

  int16_t status_;
  bool data_ready_;

  static bool trigger(repeating_timer_t *rt) {
    _pub_odom_triggered = true;
    return true;
  }
};

#endif  // PUB_ODOM_HPP