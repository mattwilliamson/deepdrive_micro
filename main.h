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
    #include <rmw_microros/rmw_microros.h>
    // #include <std_msgs/msg/int32.h>
    // #include <std_msgs/msg/int32_multi_array.h>
    #include <control_msgs/msg/mecanum_drive_controller_state.h>
    #include <diagnostic_msgs/msg/diagnostic_status.h>
    #include <sensor_msgs/msg/battery_state.h>
    #include <sensor_msgs/msg/joint_state.h>
    #include <rosidl_runtime_c/string.h>
    #include <micro_ros_utilities/string_utilities.h>

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

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      status.set(Status::Error);                                               \
      sleep_ms(10000);                                                         \
      printf("Failed status on line %d: (error code: %d) Aborting.\n",         \
             __LINE__, (int)temp_rc);                                          \
      return 1;                                                                \
    }                                                                          \
  }

#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      status.set(Status::Warning);                                             \
      sleep_ms(10000);                                                         \
      printf("Failed status on line %d: (error code: %d). Continuing.\n",      \
             __LINE__, (int)temp_rc);                                          \
    } else {                                                                   \
      status.set(Status::Success);                                             \
    }                                                                          \
  }

#define MICROSECONDS 1000000

char JOINT_NAME_FRONT_LEFT[] =    "wheel_front_left_joint";
char JOINT_NAME_BACK_LEFT[] =     "wheel_back_left_joint";
char JOINT_NAME_FRONT_RIGHT[] =   "wheel_front_right_joint";
char JOINT_NAME_BACK_RIGHT[] =    "wheel_back_right_joint";