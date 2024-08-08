#ifndef MICRO_ROS_HPP
#define MICRO_ROS_HPP

#include <Arduino.h>

#include <FreeRTOS.h>
#include <task.h>

#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float64.h>

#include "RosoutLogger.hpp"
#include "config.h"

extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;

extern RosoutLogger *logger;

void setupMicroROS();

// Error handle loop
void error_loop();

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      Serial.println("error code: " + String(temp_rc));                        \
      error_loop();                                                            \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }
// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc !=
// RCL_RET_OK)){printf("Failed status on line %d: %d.
// Aborting.\n",__LINE__,(int)temp_rc); return 1;}} #define RCSOFTCHECK(fn) {
// rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on
// line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

void setupMicroROS();
void vTaskPing(void *pvParameters);
void vTaskMicroROS(void *pvParameters);

#endif // MICRO_ROS_HPP