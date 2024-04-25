#ifndef PUB_IMU_HPP
#define PUB_IMU_HPP

#include <algorithm>
#include <cmath>
#include <vector>

extern "C" {
#include <micro_ros_utilities/string_utilities.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>

#include "constants.h"
}

#include "imu.hpp"
#include "pubsub.hpp"
#include "quaternion.hpp"

static bool _pub_imu_triggered;

class PubImu {
 public:
  int16_t get_status() {
    return status_;
  }

  PubImu(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
         int64_t timer_hz = 10,
         const char *topic_name = "~/imu",
         const char *frame_id = IMU_FRAME);

  void publish();
  void calculate();

  PubImu &operator=(const PubImu &) = default;

  ~PubImu() {
    // if (publisher_ != nullptr) {
    status_ = rcl_publisher_fini(&publisher_, node_);
    // }
  }

 private:
  rcl_node_t *node_;
  rcl_allocator_t *allocator_;
  rclc_support_t *support_;
  rclc_executor_t *executor_;

  sensor_msgs__msg__Imu *msg_;

  rcl_publisher_t publisher_;
  mutex_t lock_;
  repeating_timer_t timer_;
  IMU imu;

  int16_t status_;
  bool data_ready_;

  static bool trigger(repeating_timer_t *rt) {
    _pub_imu_triggered = true;
    return true;
  }
};

#endif  // PUB_IMU_HPP