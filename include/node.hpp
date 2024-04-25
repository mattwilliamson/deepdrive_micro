#ifndef NODE_HPP
#define NODE_HPP

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

extern "C" {
// Error codes can be found in
// micro_ros_raspberrypi_pico_sdk/libmicroros/include/rcl/types.h
// and micro_ros_raspberrypi_pico_sdk/libmicroros/include/rmw/ret_types.h
#include <control_msgs/msg/mecanum_drive_controller_state.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_utilities/string_utilities.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/sequence_bound.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/int32.h>

#include "config.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "led_status.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico_uart_transports.h"
#include "constants.h"
}

#include "analog_sensors.hpp"
#include "led_ring.hpp"
#include "motor.hpp"
#include "quaternion.hpp"
#include "status.hpp"
#include "pubsub.hpp"
#include "pub_odom.hpp"
#include "pub_imu.hpp"
#include "motor_manager.hpp"
#include "pub_telemetry.hpp"
#include "pub_joint_state.hpp"
#include "sub_cmd_vel.hpp"

/**
 * @class Node
 * @brief Represents a node in the ROS (Robot Operating System) framework.
 *
 * The Node class encapsulates the functionality of a ROS node, including publishers, subscribers, timers, and other components.
 * It provides methods for initializing and starting the control loop and the main loop, as well as various initialization methods for publishers and subscribers.
 * The class also includes error handling functions for uROS (micro ROS) operations.
 *
 * @note This class follows the Singleton design pattern, ensuring that only one instance of the Node class can exist.
 */
class Node {
 public:
  // Number of uRosHandles allocated in the executor (1 timer, 1 subscription,
  // 6 publisher)
  // TODO: Increment these memory handles
  // const size_t uRosHandles = 12 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES;
  const size_t uRosHandles = 20;

  // rcl_init_options_t init_options;
  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rclc_executor_t executor;
  rcl_timer_t timer_main_loop;
  // rclc_parameter_server_t param_server;

  rcl_publisher_t publisher_battery;
  // rcl_publisher_t publisher_join_state;
  // rcl_publisher_t publisher_mag;

  PubOdom *pub_odom;
  PubImu *pub_imu;
  PubTelemetry *pub_telemetry;
  PubJointState *pub_joint_state;

  SubCmdVel *sub_cmd_vel;

  // control_msgs__msg__MecanumDriveControllerState mgs_out_motor_speed;  // TODO: Find the create function for this
  sensor_msgs__msg__BatteryState msg_out_battery;                // TODO: Find the create function for this
  


  LEDRing led_ring = LEDRing(LED_RING_PIN, LED_RING_PIO, LED_RING_NUM_PIXELS);
  StatusManager status;

  // TODO: Do I need to publish trajectory_msgs__msg__JointTrajectoryPoint ?

  volatile bool render_led_ring = false;
  volatile bool control_due = false;

  repeating_timer_t timer_control;
  repeating_timer_t timer_led_ring;

  AnalogSensors* analog_sensors;

  void RCCHECK(rcl_ret_t error_code) {
    if (error_code != RCL_RET_OK) {
      urosErrorHandler(error_code);
    }
  }

  void RCSOFTCHECK(rcl_ret_t error_code) {
    if (error_code != RCL_RET_OK) {
      urosWarningHandler(error_code);
    }
  }

  // Singleton
  static Node& getInstance() {
    static Node instance;
    return instance;
  }

  static double ceil_radians(double rad);

  Node();


  // Run control loop in a separate core
  void spin_control_loop();
  int start_control_loop();

  /**
   * @brief Run main loop for pub/sub and other uROS stuff
   *
   * This function starts the main loop of the program.
   * It is responsible for executing the core functionality of the program.
   * Runs on core0.
   *
   * @return An integer value indicating the status of the main loop.
   */
  int init_main_loop();
  int start_main_loop();
  void spin_main_loop(rcl_timer_t* timer_main_loop, int64_t last_call_time);

  // Top level main loop
  void spin();

  MotorManager *motor_manager_;

  // Watchdog
  void init_watchdog();

  // Param server
  // int init_param_server();
  // bool on_parameter_changed(const Parameter* old_param,
  //                           const Parameter* new_param, void* context);
  // bool isDoubleNamed(const Parameter* new_param, const char* name);


  // Motor Publisher
  void init_motor_pub();
  void publish_motor();

  // Diagnostic publisher
  int init_diagnostic();
  void publish_diagnostic();

  // Battery publisher
  int init_battery();
  void publish_battery();

  // TODO: Maybe put this in a destructor
  ~Node();

  /*
   * \return #RCL_RET_OK if the client was initialized successfully, or
   * \return #RCL_RET_NODE_INVALID if the node is invalid, or
   * \return #RCL_RET_ALREADY_INIT if the client is already initialized, or
   * \return #RCL_RET_INVALID_ARGUMENT if any arguments are invalid, or
   * \return #RCL_RET_BAD_ALLOC if allocating memory fails, or
   * \return #RCL_RET_SERVICE_NAME_INVALID if the given service name is invalid,
   * or \return #RCL_RET_ERROR if an unspecified error occurs.
   * */

  void urosErrorHandler(rcl_ret_t error_code) {
    switch (error_code) {
      case RCL_RET_OK:
        printf("uROS Success\n");
        return;
      case RCL_RET_NODE_INVALID:
        printf("uROS Error: Node is invalid\n");
        break;
      case RCL_RET_ALREADY_INIT:
        printf("uROS Error: Client is already initialized\n");
        break;
      case RCL_RET_INVALID_ARGUMENT:
        printf("uROS Error: Invalid argument\n");
        break;
      case RCL_RET_BAD_ALLOC:
        printf("uROS Error: Memory allocation failed\n");
        break;
      case RCL_RET_SERVICE_NAME_INVALID:
        printf("uROS Error: Service name is invalid\n");
        break;
      case RCL_RET_ERROR:
        printf("uROS Error: Unspecified error\n");
        break;
      default:
        printf("uROS Error: Unknown error code\n");
        break;
    }
    printf("uROS Error occurred: %d\n", error_code);
    status.set(Status::Error);
    sleep_ms(1000);
  }

  void urosWarningHandler(rcl_ret_t error_code) {
    printf("uROS Error occurred: %d\n", error_code);
    status.set(Status::Warning);
    sleep_ms(1000);
  }

  // private:
};

#endif  // NODE_HPP