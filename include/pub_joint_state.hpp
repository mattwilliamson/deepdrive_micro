#ifndef PUB_JOINTSTATE_HPP
#define PUB_JOINTSTATE_HPP

#include <algorithm>
#include <cmath>
#include <vector>

extern "C" {
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/joint_state.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "constants.h"
}

#include "pubsub.hpp"
#include "quaternion.hpp"
#include "motor_manager.hpp"


static bool _pub_jointstate_triggered;

class PubJointState {
 public:
  int16_t get_status() {
    return status_;
  }

  PubJointState(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
          MotorManager *motor_manager_,
          int64_t timer_hz = 10,
          const char *topic_name = "~/joint_state",
          const char *frame_id = "base_link");

  void publish();
  void calculate();

  ~PubJointState() {
    // if (publisher_ != nullptr) {
      rcl_publisher_fini(&publisher_, node_);
    // }
  }

 private:
  rcl_node_t *node_;
  rcl_allocator_t *allocator_;
  rclc_support_t *support_;
  rclc_executor_t *executor_;
  rcl_publisher_t publisher_;
  repeating_timer_t timer_;
  mutex_t lock_;

  sensor_msgs__msg__JointState* msg_;

  MotorManager *motor_manager_;

  int16_t status_;
  bool data_ready_;

  static bool trigger(repeating_timer_t *rt) {
    _pub_jointstate_triggered = true;
    return true;
  }
};

#endif  // PUB_JOINTSTATE_HPP