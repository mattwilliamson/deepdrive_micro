#ifndef PUB_BATTERY_STATE_HPP
#define PUB_BATTERY_STATE_HPP

#include "analog_sensors.hpp"
#include "pubsub/pubsub.hpp"
#include "status.hpp"
#include "constants.h"

class PubBatteryState {
 public:
  int16_t get_status() {
    return status_;
  }

  PubBatteryState(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator,
              AnalogSensors *analog_sensors, 
               int64_t timer_hz = BATTERY_STATE_LOOP_HZ,
               const char *topic_name = "~/battery");

  void publish();
  void calculate();

  ~PubBatteryState();

 private:
  rcl_node_t *node_;
  rcl_allocator_t *allocator_;
  rclc_support_t *support_;
  rclc_executor_t *executor_;
  rcl_publisher_t publisher_;
  mutex_t lock_;
  repeating_timer_t timer_;

  sensor_msgs__msg__BatteryState *msg_;

  AnalogSensors *analog_sensors_;

  int16_t status_;
  bool data_ready_;

  static bool trigger(repeating_timer_t *rt);
};

#endif  // PUB_BATTERY_STATE_HPP