#include "UltrasonicRangePublisher.h"

/**
 * @brief Constructs an UltrasonicRangePublisher object.
 * 
 * Initializes the class with the specified ROS2 topics, sensor, and rates for publishing and measurements.
 * 
 * @param range_topic_name The name of the ROS2 topic to publish the Range data.
 * @param laser_scan_topic_name The name of the ROS2 topic to publish the LaserScan data.
 * @param sensor A reference to an HCSR04 object to retrieve sensor data.
 * @param publish_rate The rate at which to publish the data in Hz.
 * @param measurement_rate The rate at which to trigger measurements in Hz.
 * @param support A pointer to the rclc_support_t structure for ROS2 support.
 * @param node A pointer to the rcl_node_t structure for ROS2 node.
 * @param executor A pointer to the rclc_executor_t structure for ROS2 executor.
 */
UltrasonicRangePublisher::UltrasonicRangePublisher(
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
)
    : frame_id(frame_id),
      range_topic_name(range_topic_name),
      laser_scan_topic_name(laser_scan_topic_name),
      sensor(sensor),
      publish_rate(publish_rate),
      measurement_rate(measurement_rate),
      angle_offset(angle_offset),
      support(support),
      node(node),
      executor(executor),
      last_publish_time(0),
      last_measurement_time(0) {

    // https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
    laser_scan_msg = sensor_msgs__msg__LaserScan__create();
    laser_scan_msg->header.frame_id = micro_ros_string_utilities_init(frame_id);
    
    laser_scan_msg->angle_min = (-SONAR_FOV - angle_offset / 2.0) * M_PI / 180;                          // degrees in radians start of scan
    laser_scan_msg->angle_max = (SONAR_FOV + angle_offset / 2.0) * M_PI / 180;                           // degrees in radians end of scan
    laser_scan_msg->angle_increment = SONAR_FOV * M_PI / 180 / SONAR_LASER_RAYS;  // degrees in radians
    laser_scan_msg->time_increment = 0.0;                                         // time between measurements [seconds]
    laser_scan_msg->scan_time = 1.0 / measurement_rate;                           // time between scans [seconds]
    laser_scan_msg->range_min = SONAR_MIN_DISTANCE;                               // minimum range value [m]
    laser_scan_msg->range_max = SONAR_MAX_DISTANCE;                               // maximum range value [m]

    assert(rosidl_runtime_c__float32__Sequence__init(&laser_scan_msg->ranges, SONAR_LASER_RAYS));
    assert(rosidl_runtime_c__float32__Sequence__init(&laser_scan_msg->intensities, 0));

    // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Range.html

    // Initialize the LaserScan message
    // laser_scan_msg.angle_min = ANGLE_MIN_RAD;
    // laser_scan_msg.angle_max = ANGLE_MAX_RAD;
    // laser_scan_msg.angle_increment = ANGLE_INCREMENT_RAD;
    // laser_scan_msg.time_increment = TIME_INCREMENT;
    // laser_scan_msg.scan_time = SCAN_TIME_DIVISOR / publish_rate;
    // laser_scan_msg.range_min = MIN_RANGE_METERS;
    // laser_scan_msg.range_max = MAX_RANGE_METERS;

    // Initialize the Range message
    range_msg = sensor_msgs__msg__Range__create();
    range_msg->header.frame_id = micro_ros_string_utilities_init(frame_id);
    range_msg->radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    range_msg->field_of_view = FIELD_OF_VIEW_RAD;
    range_msg->min_range = MIN_RANGE_METERS;
    range_msg->max_range = MAX_RANGE_METERS;

}

/**
 * @brief Initializes the sensor and the ROS2 publishers.
 */
void UltrasonicRangePublisher::init() {
    // Initialize the sensor
    sensor.begin();

    // Initialize the Range publisher
    rclc_publisher_init_default(
        &range_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        range_topic_name
    );

    // Initialize the LaserScan publisher
    rclc_publisher_init_default(
        &laser_scan_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        laser_scan_topic_name
    );
}

/**
 * @brief Updates the sensor data and publishes it if the publish rate interval has passed.
 */
void UltrasonicRangePublisher::update() {
    unsigned long current_time = millis();

    // Trigger measurement if measurement rate interval has passed
    if (current_time - last_measurement_time >= (1000.0 / measurement_rate)) {
        sensor.update();
        last_measurement_time = current_time;
        delay(1);
    }

    // Publish data if publish rate interval has passed
    if (current_time - last_publish_time >= (1000.0 / publish_rate)) {
        float distance = sensor.getDistance();

        if (distance = SONAR_MAX_DISTANCE) {
            distance = INFINITY;
        }

        // Publish Range message
        range_msg->range = distance;
        RCSOFTCHECK(rcl_publish(&range_publisher, range_msg, NULL));

        // Publish LaserScan message
        for (size_t i = 0; i < SONAR_LASER_RAYS; i++) {
            laser_scan_msg->ranges.data[i] = distance;
        }
        RCSOFTCHECK(rcl_publish(&laser_scan_publisher, laser_scan_msg, NULL));

        last_publish_time = current_time;
    }
    delay(1);
}
