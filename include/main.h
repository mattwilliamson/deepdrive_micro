#pragma once

/*
 * C++ HEADERS
 */
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstdint>
#include <cstring>

extern "C" {
    // Error codes can be found in
    // micro_ros_raspberrypi_pico_sdk/libmicroros/include/rcl/types.h
    // and micro_ros_raspberrypi_pico_sdk/libmicroros/include/rmw/ret_types.h
    #include <rcl/error_handling.h>
    #include <rcl/rcl.h>
    #include <rclc/executor.h>
    #include <rclc/rclc.h>
    #include <rclc_parameter/rclc_parameter.h>
    #include <rmw_microros/rmw_microros.h>
    #include <rosidl_runtime_c/sequence_bound.h>
    // #include <std_msgs/msg/int32.h>
    // #include <std_msgs/msg/int32_multi_array.h>
    #include <control_msgs/msg/mecanum_drive_controller_state.h>
    #include <diagnostic_msgs/msg/diagnostic_status.h>
    #include <sensor_msgs/msg/battery_state.h>
    #include <sensor_msgs/msg/joint_state.h>
    #include <sensor_msgs/msg/imu.h>
    #include <sensor_msgs/msg/magnetic_field.h>
    #include <diagnostic_msgs/msg/diagnostic_array.h>
    #include <diagnostic_msgs/msg/key_value.h>
    #include <geometry_msgs/msg/twist.h>
    #include <nav_msgs/msg/odometry.h>
    #include <geometry_msgs/msg/point.h>
    #include <rosidl_runtime_c/string.h>
    #include <micro_ros_utilities/string_utilities.h>
    #include <rosidl_runtime_c/string_functions.h>
    #include <rosidl_runtime_c/primitives_sequence_functions.h>

    // /*
    //  * PICO HEADERS
    //  */
    // #include "pico/stdlib.h"
    // #include "pico/binary_info.h"
    // #include "hardware/gpio.h"
    // #include "hardware/i2c.h"
    // #include "hardware/spi.h"
    // #include "hardware/adc.h"
    // #include "hardware/uart.h"
    #include "hardware/pwm.h"
    #include "pico/stdlib.h"
    #include "pico_uart_transports.h"
    #include "pico/multicore.h"
    #include "pico/time.h"
    #include "hardware/watchdog.h"

    #include "led_status.h"
}

#include "config.h"
#include "status.h"
#include "led_ring.h"
#include "motor.h"
#include "analog_sensors.h"
#include "hardware/i2c.h"
#include "imu.h"

StatusManager& status = StatusManager::getInstance();

/*
 * \return #RCL_RET_OK if the client was initialized successfully, or
 * \return #RCL_RET_NODE_INVALID if the node is invalid, or
 * \return #RCL_RET_ALREADY_INIT if the client is already initialized, or
 * \return #RCL_RET_INVALID_ARGUMENT if any arguments are invalid, or
 * \return #RCL_RET_BAD_ALLOC if allocating memory fails, or
 * \return #RCL_RET_SERVICE_NAME_INVALID if the given service name is invalid, or
 * \return #RCL_RET_ERROR if an unspecified error occurs.
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

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      urosErrorHandler(temp_rc);                                               \
      sleep_ms(1000);                                                          \
      printf("Failed status on line %d: (error code: %d) Aborting.\n",         \
             __LINE__, (int)temp_rc);                                          \
      return 1;                                                                \
    }                                                                          \
  }

#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      urosWarningHandler(temp_rc);                                             \
      printf("Failed status on line %d: (error code: %d). Continuing.\n",      \
             __LINE__, (int)temp_rc);                                          \
    } else {                                                                   \
      status.set(Status::Success);                                             \
    }                                                                          \
  }

#define MICROSECONDS 1e6
#define MILLISECONDS 1e3

const char *PARAM_PID_KP = "pid_k_p";
const char *PARAM_PID_KI = "pid_k_i";
const char *PARAM_PID_KD = "pid_k_d";