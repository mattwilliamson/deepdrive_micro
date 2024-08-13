#include "RosoutLogger.h"

RosoutLogger::RosoutLogger(rcl_node_t* node, rclc_support_t* support)
: publisher(rcl_get_zero_initialized_publisher())
{
    initialize_publisher(node);

    log_msg.level = rcl_interfaces__msg__Log__INFO;
    log_msg.name.data = (char*)malloc(ARRAY_LEN * sizeof(char));
    log_msg.name.size = 0;
    log_msg.name.capacity = ARRAY_LEN;
    log_msg.msg.data = (char*)malloc(ARRAY_LEN * sizeof(char));
    log_msg.msg.size = 0;
    log_msg.msg.capacity = ARRAY_LEN;
}

RosoutLogger::~RosoutLogger()
{
    RCCHECK(rcl_publisher_fini(&publisher, nullptr));
    free(log_msg.name.data);
    free(log_msg.msg.data);
}

void RosoutLogger::initialize_publisher(rcl_node_t* node)
{
    RCCHECK(rclc_publisher_init(
        &publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
        "/rosout",
        &rcl_qos_profile_rosout_default));
}

void RosoutLogger::publish_log_message(const char* message, uint8_t level)
{
    snprintf(log_msg.name.data, ARRAY_LEN, "deepdrive_micro");
    log_msg.name.size = strlen(log_msg.name.data);
    snprintf(log_msg.msg.data, ARRAY_LEN, "%s #%d", message, counter++);
    log_msg.msg.size = strlen(log_msg.msg.data);
    log_msg.level = level;

    RCSOFTCHECK(rcl_publish(&publisher, &log_msg, nullptr));
}

void RosoutLogger::Debug(const String& message)
{
    publish_log_message(message.c_str(), rcl_interfaces__msg__Log__DEBUG);
}

void RosoutLogger::Info(const String& message)
{
    publish_log_message(message.c_str(), rcl_interfaces__msg__Log__INFO);
}

void RosoutLogger::Warn(const String& message)
{
    publish_log_message(message.c_str(), rcl_interfaces__msg__Log__WARN);
}

void RosoutLogger::Error(const String& message)
{
    publish_log_message(message.c_str(), rcl_interfaces__msg__Log__ERROR);
}

void RosoutLogger::Fatal(const String& message)
{
    publish_log_message(message.c_str(), rcl_interfaces__msg__Log__FATAL);
}

void RosoutLogger::error_loop()
{
    while (1) {
        delay(100);
    }
}
