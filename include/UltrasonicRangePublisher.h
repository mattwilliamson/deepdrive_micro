#ifndef ULTRASONIC_RANGE_PUBLISHER_H
#define ULTRASONIC_RANGE_PUBLISHER_H

#include <Arduino.h>
#include <MicroROS.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/laser_scan.h>
#include "HCSR04.h"
#include "config.h"

// Define constants as macros
#define FIELD_OF_VIEW_RAD 0.261799  // 15 degrees in radians
#define ANGLE_MIN_RAD -0.261799     // -15 degrees in radians
#define ANGLE_MAX_RAD 0.261799      // 15 degrees in radians
#define ANGLE_INCREMENT_RAD 0.523599  // 30 degrees in radians (assuming only 1 reading for HC-SR04)
#define TIME_INCREMENT 0.000001      // A very small time increment value for the LaserScan message
#define SCAN_TIME_DIVISOR 1.0        // The divisor for scan_time calculation

/**
 * @class UltrasonicRangePublisher
 * @brief A class that publishes distance data from the HC-SR04 sensor as both ROS2 Range and LaserScan messages.
 */
class UltrasonicRangePublisher {
public:
    /**
     * @brief Constructor for the UltrasonicRangePublisher class.
     * 
     * @param frame_id The frame ID for the Range and LaserScan messages.
     * @param range_topic_name The name of the ROS2 topic to publish the Range data.
     * @param laser_scan_topic_name The name of the ROS2 topic to publish the LaserScan data.
     * @param sensor A reference to an HCSR04 object to retrieve sensor data.
     * @param publish_rate The rate at which to publish the data in Hz.
     * @param measurement_rate The rate at which to trigger measurements in Hz.
     * @param angle_offset The offset angle for the sensor in radians.
     * @param support A pointer to the rclc_support_t structure for ROS2 support.
     * @param node A pointer to the rcl_node_t structure for ROS2 node.
     * @param executor A pointer to the rclc_executor_t structure for ROS2 executor.
     */
    UltrasonicRangePublisher(
        const char* frame_id,
        const char* range_topic_name,
        const char* laser_scan_topic_name,
        HCSR04& sensor,
        float publish_rate,
        float measurement_rate,
        float angle_offset,
        rclc_support_t* support,
        rcl_node_t* node,
        rclc_executor_t* executor
    );

    /**
     * @brief Initializes the publishers and sensor.
     */
    void init();

    /**
     * @brief Updates the sensor data and publishes it if the publish rate interval has passed.
     */
    void update();

private:
    const char* frame_id;
    rcl_publisher_t range_publisher;
    rcl_publisher_t laser_scan_publisher;
    sensor_msgs__msg__Range *range_msg;
    sensor_msgs__msg__LaserScan *laser_scan_msg;
    const char* range_topic_name;
    const char* laser_scan_topic_name;
    HCSR04& sensor;
    float publish_rate;
    float measurement_rate;
    float angle_offset;
    unsigned long last_publish_time;
    unsigned long last_measurement_time;
    rclc_support_t* support;
    rcl_node_t* node;
    rclc_executor_t* executor;
};

#endif
