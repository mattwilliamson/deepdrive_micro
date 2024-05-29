#ifndef MOTOR_H
#define MOTOR_H

#include <math.h>

#include <limits>
#include <map>

#include <rmw_microros/rmw_microros.h>

extern "C" {
#include "config.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "config.h"
}

#include "pid.hpp"
#include "ring_buffer.hpp"

using Micrometers = int64_t;
using Meters = double;
using Pulses = int64_t;
using DutyCycle = int16_t;
using Radians = double;
using Side = uint8_t;

// Keep the intermediate pulse counts until the next read from the map

// Flag to check if the GPIO callbacks are configured
static bool gpio_callbacks_configured = false;

// TODO: Prune this down to use less memory, an array is fast for now, just wasting some ints in memory

// Map of pin number to pulse count
static volatile uint32_t pulse_count_map[30] = {};

// Got some help from https://cocode.se/linux/raspberry/pwm.html

/**
 * @class Motor
 * @brief Represents a motor with control functions.
 *
 * The Motor class provides methods to control the speed and state of a motor.
 * It can be used to control various types of motors, such as DC motors or
 * servo motors.
 */
class Motor {
 public:
  // 1000 us
  static const DutyCycle DUTY_CYCLE_MIN = MOTOR_DUTY_CYCLE_MIN;
  // 1500 us
  static const DutyCycle DUTY_CYCLE_CENTER = MOTOR_DUTY_CYCLE_CENTER;
  // 2000 us
  static const DutyCycle DUTY_CYCLE_MAX = MOTOR_DUTY_CYCLE_MAX;

  // diameter of the wheel in micrometers
  static const Micrometers WHEEL_DIAMETER = WHEEL_DIAMETER_MM * MICRO_METERS / MILLI_METERS;
  static const Micrometers WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;

  // distance between left and right wheels
  static constexpr Micrometers WHEEL_BASE = WHEEL_BASE_COEFFICIENT * WHEEL_BASE_MM * MICRO_METERS / MILLI_METERS;

  // Pulse count per revolution of the motor
  static const Pulses PULSES_PER_REV = MOTOR_PULSES_PER_REV;  // 2x for rising and falling edge

  // Pulses per second at full throttle (of the slowest motor)
  static const Pulses MAX_SPEED_PPS = MOTOR_MAX_SPEED_PPS;  // 2x for rising and falling edge

  // Micrometers per revolution of the motor
  static constexpr Micrometers UM_PER_REV = WHEEL_DIAMETER * M_PI;

  // Number of pulses at max speed for each time slice of the control loop
  static constexpr Pulses MAX_PULSES_PER_LOOP = MAX_SPEED_PPS / CONTROL_LOOP_HZ;

  // static constexpr Pulses PULSES_PER_UM = PULSES_PER_REV / UM_PER_REV;
  // static constexpr double MAX_SPEED_US_PER = MAX_SPEED_PPS / PULSES_PER_UM;
  // // Meter per second at full throttle

  PIDController *pidController_;  // PID controller for the motor

  /**
   * @brief Constructor for Motor class.
   * @param pin The pin number to which the motor is connected.
   * @param encoderPin The pin number to which the motor encoder is connected.
   */
  Motor(Side side, int pin, int encoderPin);

  /**
   * @brief Start the motor.
   */
  void start();

  /**
   * @brief Stop the motor.
   */
  void stop();

  /**
   * @brief Enable the motor.
   */
  void enable();

  /**
   * @brief Disable the motor.
   */
  void disable();

  /**
   * @brief Reset the pulse count of the motor to zero.
   */
  void reset_pulses() {
    pulses_ = 0;
    pidController_->reset();
  }

  /**
   * @brief Get the side of the motor.
   * @return The side of the motor. MOTOR_LEFT or MOTOR_RIGHT.
   */
  Side get_side() { return side_; }

  /**
   * @brief Map a value from one range to another.
   * @param value The value to be mapped.
   * @param fromMin The minimum value of the input range.
   * @param fromMax The maximum value of the input range.
   * @param toMin The minimum value of the output range.
   * @param toMax The maximum value of the output range.
   * @return The mapped value.
   */
  static int64_t map(int64_t value, int64_t fromMin, int64_t fromMax,
                     int64_t toMin, int64_t toMax) {
    // Calculate the range of the input and output values
    int64_t inputRange = fromMax - fromMin;
    int64_t outputRange = toMax - toMin;

    // Map the value from the input range to the output range
    int64_t mappedValue =
        ((value - fromMin) * outputRange) / inputRange + toMin;

    return mappedValue;
  }

  // Convert pulses/sec to max/min duty cycle
  static DutyCycle pulsesToDutyCycle(Pulses pulsesPerSecond) {
    return map(pulsesPerSecond, -MAX_SPEED_PPS, MAX_SPEED_PPS, DUTY_CYCLE_MIN,
               DUTY_CYCLE_MAX);
  }

  // Convert Micrometers to pulses
  static Pulses micrometersToPulses(Micrometers uM) {
    return uM * PULSES_PER_REV / UM_PER_REV;
  }

  // Convert pulses to micrometers
  static Micrometers pulsesToMicrometers(Pulses pulses) {
    return (Micrometers)pulses * UM_PER_REV / PULSES_PER_REV;
  }

  // Convert pulses to meters
  static Meters pulsesToMeters(Pulses pulses) {
    Micrometers um = pulsesToMicrometers(pulses);
    return (Meters)um / MICRO_METERS;
  }

  /**
   * @brief Get the speed of the motor commanded by the PID controller.
   * @return The speed of the motor in pulses per second.
   */
  Pulses getSpeedSignal() {
    // TODO: Calculate upon update and just read
    return speedSignal_;
  }

  void setSpeedSignal(Pulses speedSignal);

  /**
   * @brief Get the current actual speed of the motor in Micrometers per
   * second.
   * @return The speed of the motor in Micrometers per second.
   */
  Micrometers getSpeedMicrometers() {
    // TODO: Calculate upon update and just read
    return pulsesToMicrometers(speed_);
  }

  /**
   * @brief Get the current actual speed of the motor in Micrometers per
   * loop.
   * @return The speed of the motor in Micrometers per loop.
   */
  Micrometers getMicrometersLoop() {
    return pulsesToMicrometers(pulses_loop_);
  }

  /**
   * @brief Get the current actual speed of the motor in Meters per
   * loop.
   * @return The speed of the motor in Meters per loop.
   */
  Micrometers getMetersLoop() {
    Meters um = pulsesToMicrometers(pulses_loop_);
    return um / MICRO_METERS;
  }

  /**
   * @brief Get the current actual speed of the motor in Meters per second.
   * @return The speed of the motor in Meters per second.
   */
  Meters getSpeedMeters() {
    Meters um = pulsesToMicrometers(speed_);
    return um / MICRO_METERS;
  }

  /**
   * @brief Get the current actual speed of the motor in pulses per second.
   * @return The speed of the motor in pulses per second.
   */
  Pulses getSpeed() {
    return speed_;
  }

  Radians getPosition() {
    // TODO: Calculate upon update and just read
    return pulsesToRadians(pulses_);
  }

  Radians getSpeedRadians() {
    // TODO: Calculate upon update and just read
    return pulsesToRadians(speed_);
  }

  /**
   * @brief Set the speed of the motor.
   * @param speed The speed value to set for the motor.
   *        Negative values indicate reverse, 0 is stopped, and the maximum
   * value is the maximum integer value (-/+32767).
   */
  // void setSpeed(Pulses speed);

  /**
   * @brief Set the target speed of the motor.
   * @param targetSpeed The target speed value to set for the motor in pulses
   * per second. Negative values indicate reverse, 0 is stopped.
   */
  void setTargetSpeed(Pulses targetSpeed);

  void setTargetSpeedMeters(Meters metersPerSecond) {
    Pulses p = micrometersToPulses(metersPerSecond * MICRO_METERS);
    setTargetSpeed(p);
  }

  /**
   * @brief Get the target speed of the motor.
   * @return The target speed of the motor in pulses/sec.
   */
  Pulses getTargetSpeed();

  /**
   * @brief Get the target speed of the motor.
   * @return The target speed of the motor in meters/sec.
   */
  Meters getTargetSpeedMeters();

  /**
   * @brief Read the number of pulses counted by the motor.
   */
  void readPulses();

  /**
   * @brief Get the total number of pulses counted by the motor.
   * @return The number of pulses counted by the motor.
   */
  Pulses getPulses() { return pulses_; }

  Micrometers getTotalMicrometers() {
    return pulsesToMicrometers(pulses_);
  }

  Meters getTotalMeters() {
    Meters um = pulsesToMicrometers(pulses_);
    return um / MICRO_METERS;
  }

  /**
   * @brief Calculate the PID controller output based on the desired speed and
   * the current speed. Should only be called once per time period (control
   * loop).
   * @return The PID controller output in the range of pulses/sec.
   */
  int16_t calculatePid();

  /**
   * @brief Convert pulses to radians.
   * @param pulses The number of pulses to convert.
   * @return The equivalent value in radians.
   */
  static Radians pulsesToRadians(Pulses pulses) {
    double conversionFactor = 2.0 * M_PI / PULSES_PER_REV;
    return pulses * conversionFactor;
  }

  /**
   * @brief Convert radians to pulses.
   * @param radians The number of radians to convert.
   * @return The equivalent value in pulses.
   */
  static Pulses radiansToPulses(Radians radians) {
    double conversionFactor = PULSES_PER_REV / (2.0 * M_PI);
    return radians * conversionFactor;
  }

  void updateMotorOutput();

 private:
  int pin_;                                                 // Pin number of the motor
  int encoderPin_;                                          // Pin number of the motor encoder
  int64_t speed_;                                           // current speed pulses/sec
  int64_t pulses_loop_;                                     // current speed pulses/loop
  int64_t speedSignal_;                                     // current speed signal pulses/sec
  int64_t targetSpeed_;                                     // desired speed of the motor pulses/sec
  int64_t pulses_;                                          // Number of pulses counted by the motor total
  int dutyCycle_;                                           // PWM duty cycle
  uint slice_;                                              // PWM slice number
  uint channel_;                                            // PWM channel number
  int8_t direction_;                                        // Direction of the motor (1, 0, -1)
  Side side_;                                               // Which side the motor is on
  RingBuffer<uint64_t, ENCODER_PULSE_BUFFER> pulseBuffer_;  // Buffer to store pulse counts
  uint64_t lastRead_;                                       // Last time the encoder was read

  int64_t enabled_time_;
  bool enabled_ = false;
};

#endif  // MOTOR_H