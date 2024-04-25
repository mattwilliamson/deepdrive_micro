#include "motor_manager.hpp"

MotorManager::MotorManager() {
    motors_.resize(MOTOR_COUNT);

  // Setup 4 ESC brushless motor controllers
  motors_[IDX_MOTOR_FRONT_LEFT] = new Motor(MOTOR_LEFT, PIN_MOTOR_FRONT_LEFT, PIN_ENCODER_FRONT_LEFT);
  motors_[IDX_MOTOR_BACK_LEFT] = new Motor(MOTOR_LEFT, PIN_MOTOR_BACK_LEFT, PIN_ENCODER_BACK_LEFT);
  motors_[IDX_MOTOR_FRONT_RIGHT] = new Motor(MOTOR_RIGHT, PIN_MOTOR_FRONT_RIGHT, PIN_ENCODER_FRONT_RIGHT);
  motors_[IDX_MOTOR_BACK_RIGHT] = new Motor(MOTOR_RIGHT, PIN_MOTOR_BACK_RIGHT, PIN_ENCODER_BACK_RIGHT);
}

void MotorManager::disable_motors() {
  for (auto &motor : motors_) {
    motor->disable();
  }
}

void MotorManager::enable_motors() {
  for (auto &motor : motors_) {
    motor->enable();
  }
}

void MotorManager::update_motor_outputs() {
    for (Motor* motor : motors_) {
      motor->updateMotorOutput();
    }
  }