#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>

/**
 * @brief A PID controller class for controlling a process variable based on a
 * setpoint.
 */
class PIDController {
 public:
  /**
   * @brief Constructor for PIDController class.
   * @param min The minimum value of the manipulated variable.
   * @param max The maximum value of the manipulated variable.
   * @param Kp The proportional gain.
   * @param Ki The integral gain.
   * @param Kd The derivative gain.
   */
  PIDController(double min, double max, double Kp, double Ki, double Kd)
      : min_(min), max_(max), Kp_(Kp), Ki_(Ki), Kd_(Kd) {
    integral_ = 0;
    pre_error_ = 0;
  }

  /**
   * @brief Overloaded constructor for PIDController class.
   * @param Kp The proportional gain.
   * @param Ki The integral gain.
   * @param Kd The derivative gain.
   */
  PIDController(double Kp, double Ki, double Kd)
      : min_(std::numeric_limits<double>::lowest()),
        max_(std::numeric_limits<double>::max()),
        Kp_(Kp),
        Ki_(Ki),
        Kd_(Kd) {
    integral_ = 0;
    pre_error_ = 0;
  }

  /**
   * @brief Default constructor for PIDController class.
   * Initializes the PID controller with default values.
   */
  PIDController()
      : min_(std::numeric_limits<double>::lowest()),
        max_(std::numeric_limits<double>::max()),
        Kp_(0.0),
        Ki_(0.0),
        Kd_(0.0) {
    integral_ = 0;
    pre_error_ = 0;
  }

  /**
   * @brief Calculates the control output based on the process variable.
   * @param dt The time difference between the current and previous time.
   * @param pv The current value of the process variable.
   * @return The control output.
   */
  double calculate(double dt, double pv);

  /**
   * @brief Calculates the control output based on the process variable.
   * @param pv The current value of the process variable.
   * @return The control output.
   */
  double calculate(double pv);

  /**
   * @brief Resets the internal state of the PID controller.
   * @details A good time to reset the PID controller is when the motors are
   * stopped.
   */
  void reset();

  /**
   * @brief Setter for the setpoint.
   * @param setpoint The new value for the setpoint.
   */
  void setSetpoint(double setpoint);

  /**
   * @brief Setter for the proportional gain (Kp).
   * @param Kp The new value for the proportional gain.
   */
  void setKp(double Kp);

  /**
   * @brief Setter for the integral gain (Ki).
   * @param Ki The new value for the integral gain.
   */
  void setKi(double Ki);

  /**
   * @brief Setter for the derivative gain (Kd).
   * @param Kd The new value for the derivative gain.
   */
  void setKd(double Kd);

  /**
   * @brief Setter for the minimum value of output.
   * @param min The new value for the minimum value of output.
   */
  void setMin(double min);

  /**
   * @brief Setter for the maximum value of output.
   * @param max The new value for the maximum value of output.
   */
  void setMax(double max);

  /**
   * @brief Destructor for PIDController class.
   */
  ~PIDController();

 private:
  double setpoint_;  // desired value for the output
  double min_;       // minimum value of output
  double max_;       // maximum value of output
  double Kp_;        // proportional gain
  double Ki_;        // integral gain
  double Kd_;        // derivative gain
  double pre_error_;
  double integral_;
};

#endif  // PID_CONTROLLER_HPP