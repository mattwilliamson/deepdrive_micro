#include "param_server.hpp"

#include "node.hpp"

// Check if strings are the same
bool Node::isDoubleNamed(const Parameter *new_param, const char *name) {
  if (new_param == NULL) {
    return false;
  }
  return strcmp(new_param->name.data, name) == 0 &&
         new_param->value.type == RCLC_PARAMETER_DOUBLE;
}

bool Node::on_parameter_changed(const Parameter *old_param,
                                const Parameter *new_param, void *context) {
  (void)context;
  if (new_param == NULL) {
    return false;
  }

  if (isDoubleNamed(new_param, PARAM_PID_KP)) {
    for (auto &motor : motors) {
      motor->pidController_->setKp(new_param->value.double_value);
    }
  } else if (isDoubleNamed(new_param, PARAM_PID_KI)) {
    for (auto &motor : motors) {
      motor->pidController_->setKi(new_param->value.double_value);
    }
  } else if (isDoubleNamed(new_param, PARAM_PID_KD)) {
    for (auto &motor : motors) {
      motor->pidController_->setKd(new_param->value.double_value);
    }
  } else {
    // Unknown param
    return false;
  }

  // int64_t old;
  // RCSOFTCHECK(rcl_timer_exchange_period(&timer,
  // RCL_MS_TO_NS(new_param->value.integer_value), &old)); printf("Publish rate
  // %ld ms\n", new_param->value.integer_value);
  return true;
}

// int Node::init_param_server() {
  // TODO: Increment memory handles
  // const size_t uRosHandles = 12 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES;

  // TODO: Figure out why param server isn't working
  // https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk/issues/925
  // https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk/blob/e315a376cacb80fe2bddaa2b3028425e0cfa4dd1/libmicroros/include/rmw_microxrcedds_c/config.h#L55
  // https://micro.ros.org/docs/tutorials/programming_rcl_rclc/parameters/
  // https://stackoverflow.com/questions/67563340/how-to-pass-command-line-arguments-to-cmake-in-vscode
  // Create parameter service
  // const rclc_parameter_options_t param_server_options = {
  //     .notify_changed_over_dds = true,
  //     .max_params = 3,
  //     .allow_undeclared_parameters = true,
  //     .low_mem_mode = true
  //   };

  // rclc_parameter_server_init_with_option(&param_server, &node,
  // &param_server_options); rclc_parameter_server_init_default(&param_server,
  // &node);

  // TODO: Fix this. Probably memory issue.
  // RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server,
  // on_parameter_changed));

  // // Add parameters
  // rclc_add_parameter(&param_server, PARAM_PID_KP, RCLC_PARAMETER_DOUBLE);
  // rclc_add_parameter(&param_server, PARAM_PID_KI, RCLC_PARAMETER_DOUBLE);
  // rclc_add_parameter(&param_server, PARAM_PID_KD, RCLC_PARAMETER_DOUBLE);

  // // Add parameters constraints
  // rclc_add_parameter_description(&param_server, PARAM_PID_KP, "Motor PID
  // Controller Kp value", ""); rclc_add_parameter_description(&param_server,
  // PARAM_PID_KI, "Motor PID Controller Ki value", "");
  // rclc_add_parameter_description(&param_server, PARAM_PID_KD, "Motor PID
  // Controller Kd value", "");

  // // Set parameter initial values
  // rclc_parameter_set_double(&param_server, PARAM_PID_KP, 0.1);
  // rclc_parameter_set_double(&param_server, PARAM_PID_KI, 0.0);
  // rclc_parameter_set_double(&param_server, PARAM_PID_KD, 0.0);

  // return 0;
// }