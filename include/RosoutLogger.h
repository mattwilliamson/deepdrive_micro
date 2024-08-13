#ifndef ROSOUT_LOGGER_HPP
#define ROSOUT_LOGGER_HPP

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl_interfaces/msg/log.h>
#include <rcl/logging_rosout.h>

#include <Arduino.h>
#include "config.h"

#define ARRAY_LEN 200

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){SerialDebug.println("error code: " + String(temp_rc)); error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

class RosoutLogger {
public:
    RosoutLogger(rcl_node_t* node, rclc_support_t* support);
    ~RosoutLogger();

    void Debug(const String& message);
    void Info(const String& message);
    void Warn(const String& message);
    void Error(const String& message);
    void Fatal(const String& message);

private:
    rcl_publisher_t publisher;
    rcl_interfaces__msg__Log log_msg;
    int counter = 0;

    void publish_log_message(const char* message, uint8_t level);
    void initialize_publisher(rcl_node_t* node);
    void error_loop();
};

#endif // ROSOUT_LOGGER_HPP
