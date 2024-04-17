#include "pid.hpp"

void PIDController::setSetpoint(double setpoint) { setpoint_ = setpoint; }

void PIDController::setKp(double Kp) { Kp_ = Kp; }

void PIDController::setKi(double Ki) { Ki_ = Ki; }

void PIDController::setKd(double Kd) { Kd_ = Kd; }

void PIDController::setMin(double min) { min_ = min; }

void PIDController::setMax(double max) { max_ = max; }

double PIDController::calculate(double pv) { return calculate(1.0, pv); }

double PIDController::calculate(double dt, double pv) {
  // error
  double error = setpoint_ - pv;

  // Proportional portion
  double Pout = Kp_ * error;

  // Integral portion
  integral_ += error * dt;
  double Iout = Ki_ * integral_;

  // Derivative portion
  double derivative = (error - pre_error_) / dt;
  double Dout = Kd_ * derivative;

  // Total output
  double output = Pout + Iout + Dout;

  // Limit to max/min
  if (output > max_)
    output = max_;
  else if (output < min_)
    output = min_;

  // Save error to previous error
  pre_error_ = error;

  return output;
}

void PIDController::reset() {
  pre_error_ = 0.0;
  integral_ = 0.0;
}

PIDController::~PIDController() {}