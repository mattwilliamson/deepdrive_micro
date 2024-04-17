#include "node.hpp"

int Node::init_joint_state() {
  // static rmw_subscription_allocation_t * allocation;

  RCCHECK(rclc_publisher_init_default(
      &publisher_join_state, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "~/joint_state"));

  // #define ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgSubfolder, MsgName) \
//   ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME( \
//     rosidl_typesupport_c, PkgName, MsgSubfolder, MsgName)()

  //   RCCHECK(rmw_init_subscription_allocation(
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
  //     ROSIDL_GET_SEQUENCE_BOUNDS(sensor_msgs, msg, JointState),
  //     &allocation
  //     ));

  msg_out_joint_state = sensor_msgs__msg__JointState__create();
  msg_out_joint_state->header.frame_id = micro_ros_string_utilities_init("base_link");

  msg_out_joint_state->name = *rosidl_runtime_c__String__Sequence__create(MOTOR_COUNT);
  rosidl_runtime_c__float64__Sequence__init(&msg_out_joint_state->position,  MOTOR_COUNT);
  rosidl_runtime_c__float64__Sequence__init(&msg_out_joint_state->velocity, MOTOR_COUNT);
  rosidl_runtime_c__float64__Sequence__init(&msg_out_joint_state->effort, MOTOR_COUNT);

  msg_out_joint_state->name.data[IDX_MOTOR_FRONT_LEFT] = micro_ros_string_utilities_init(MOTOR_JOIN_FRONT_LEFT);
  msg_out_joint_state->name.data[IDX_MOTOR_BACK_LEFT] = micro_ros_string_utilities_init(MOTOR_JOIN_BACK_LEFT);
  msg_out_joint_state->name.data[IDX_MOTOR_FRONT_RIGHT] = micro_ros_string_utilities_init(MOTOR_JOIN_FRONT_RIGHT);
  msg_out_joint_state->name.data[IDX_MOTOR_BACK_RIGHT] = micro_ros_string_utilities_init(MOTOR_JOIN_BACK_RIGHT);

  return 0;
}

void Node::publish_joint_state() {
  /**
   * The state of each joint (revolute or prismatic) is defined by:
   *  * the position of the joint (rad or m),
   *  * the velocity of the joint (rad/s or m/s) and
   *  * the effort that is applied in the joint (Nm or N).
   */
  msg_out_joint_state->header.stamp.sec = rmw_uros_epoch_millis() / MILLISECONDS;
  msg_out_joint_state->header.stamp.nanosec = rmw_uros_epoch_nanos();

  for (int i = 0; i < MOTOR_COUNT; i++) {
    msg_out_joint_state->position.data[i] = motors[i]->getPosition(); // Exception here
    msg_out_joint_state->velocity.data[i] = motors[i]->getSpeedRadians();
    // msg_out_joint_state->effort.data[i] =
    // motors[i]->getSpeedMetersPerSecond();
  }

  RCSOFTCHECK(rcl_publish(&publisher_join_state, msg_out_joint_state, NULL));
}
