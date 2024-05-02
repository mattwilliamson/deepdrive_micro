#ifndef NODE_HPP
#define NODE_HPP



extern "C" {
#include "config.h"
#include "led_status.h"
#include "pico_uart_transports.h"
#include "constants.h"
}

#include "analog_sensors.hpp"
#include "led_ring.hpp"
#include "motor.hpp"
#include "status.hpp"
#include "buzzer.hpp"
#include "pubsub/pub_odom.hpp"
#include "pubsub/pub_imu.hpp"
#include "motor_manager.hpp"
#include "pubsub/pub_telemetry.hpp"
#include "pubsub/pub_joint_state.hpp"
#include "pubsub/pub_battery_state.hpp"
#include "pubsub/pub_wheel_speed.hpp"
#include "pubsub/sub_cmd_vel.hpp"
#include "pubsub/sub_wheel_speed.hpp"
#include "pubsub/pub_sonar.hpp"

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
  
  PubOdom *pub_odom;
  PubImu *pub_imu;
  PubTelemetry *pub_telemetry;
  PubJointState *pub_joint_state;
  PubBatteryState *pub_battery_state;
  PubWheelSpeed *pub_wheel_speed;
  PubSonar *pub_sonar;

  SubCmdVel *sub_cmd_vel;
  SubWheelSpeed *sub_wheel_speed;

  LEDRing led_ring = LEDRing(LED_RING_PIN, LED_RING_PIO, LED_RING_NUM_PIXELS);
  StatusManager status;
  AnalogSensors* analog_sensors;
  Buzzer* buzzer;

  // TODO: Do I need to publish trajectory_msgs__msg__JointTrajectoryPoint ?

  volatile bool render_led_ring = false;
  volatile bool control_due = false;

  repeating_timer_t timer_control;
  repeating_timer_t timer_led_ring;

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