#ifndef PUB_TELEMETRY_HPP
#define PUB_TELEMETRY_HPP

#include <algorithm>
#include <cmath>
#include <vector>

extern "C" {
#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <micro_ros_utilities/string_utilities.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "constants.h"
}

#include "analog_sensors.hpp"
#include "motor.hpp"
#include "motor_manager.hpp"
#include "pubsub.hpp"
#include "quaternion.hpp"
#include "status.hpp"

static uint64_t core_start[2] = {0, 0};
static uint64_t core_elapsed[2] = {0, 0};

namespace Diagnostics {
const int IMU = 0;
const int CORE_0 = 1;
const int CORE_1 = 2;
}  // namespace Diagnostics

static bool _pub_telemetry_triggered;

class PubTelemetry {
 public:
  int16_t get_status() {
    return status_;
  }

  PubTelemetry(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
               MotorManager *motor_manager,
               int64_t timer_hz = TELEMETRY_LOOP_HZ,
               const char *topic_name = "/diagnostics");

  void publish();
  void calculate();

  ~PubTelemetry() {
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
  mutex_t lock_;
  repeating_timer_t timer_;
  diagnostic_msgs__msg__DiagnosticArray *msg_;

  MotorManager *motor_manager_;
  int16_t status_;
  bool data_ready_;

  static bool trigger(repeating_timer_t *rt) {
    _pub_telemetry_triggered = true;
    return true;
  }
};

#endif  // PUB_TELEMETRY_HPP