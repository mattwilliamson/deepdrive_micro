#ifndef MOTOR_MANAGER_HPP
#define MOTOR_MANAGER_HPP

#include <vector>

#include "motor.hpp"

/**
 * @brief The MotorManager class manages a collection of motors.
 *
 * The MotorManager class provides functionality to enable or disable a group of motors.
 * It also stores a vector of Motor objects.
 */
class MotorManager {
 public:
  /**
   * @brief Constructs a MotorManager object.
   */
  MotorManager();

  /**
   * @brief Disables all the motors.
   */
  void disable_motors();

  /**
   * @brief Enables all the motors.
   */
  void enable_motors();

  /**
   * @brief Updates the motor output for all motors.
   * @param output The new motor output value.
   */
  void update_motor_outputs();

  /**
   * @brief Returns the vector of motors.
   * @return A reference to the vector of motors.
   */
  std::vector<Motor*>& get_motors() {
    return motors_;
  }

 private:
  /**
   * @brief A vector of pointers to Motor objects.
   */
  std::vector<Motor*> motors_;
};

#endif  // MOTOR_MANAGER_HPP