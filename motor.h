#pragma once

#include "config.h"
#include <limits>
#include <map>
#include <math.h>
#include "pid.h"

extern "C" {
    #include "hardware/pwm.h"
    #include "pico/stdlib.h"
}


// Keep the intermediate pulse counts until the next read from the map
static bool gpio_callbacks_configured = false; // Flag to check if the GPIO callbacks are configured

// TODO: Prune this down to use less memory, an array is fast for now, just wasting some ints in memory
static volatile int pulse_count_map[30] = {}; // Map of pin number to pulse count

// Got some help from https://cocode.se/linux/raspberry/pwm.html

/**
 * @class Motor
 * @brief Represents a motor with control functions.
 * 
 * The Motor class provides methods to control the speed and state of a motor.
 * It can be used to control various types of motors, such as DC motors or servo motors.
 */
class Motor {
public:
    // Arbitrary signal range for min-max motor speed, converted to PWM duty cycle downstream
    static const int MIN_INT = std::numeric_limits<int16_t>::min() + 1; // +1 to avoid overflow
    static const int MAX_INT = std::numeric_limits<int16_t>::max() - 1; // -1 to avoid overflow

    // 1000 us
    static const int DUTY_CYCLE_MIN         = 490;
    // 1500 us
    static const int DUTY_CYCLE_CENTER      = 735;
    // 2000 us
    static const int DUTY_CYCLE_MAX         = 980;

    // TODO: There is some backlash in the motor, so we need to add some deadband (~696-688=8 pulses)
    // Forward: 34826/50 = 696 | 93156/127 = 734 | 34942/50.2 = 696 | 13988/20.1 = 695.5
    // Reverse: -34727/50.4 = -689 | -27862/40.5 = -687
    static constexpr double PULSES_PER_REV = 696;
    static constexpr double MAX_SPEED_PPS = 1450; // Pulses per second at full throttle
    static constexpr double METERS_PER_REV = WHEEL_DIAMETER * M_PI / 1000.0;
    static constexpr double REVS_PER_METER = 1.0 / METERS_PER_REV;
    static constexpr double PULSES_PER_METER = PULSES_PER_REV * REVS_PER_METER;
    static constexpr double MAX_SPEED_MS = MAX_SPEED_PPS / PULSES_PER_METER; // Meter per second at full throttle
    static constexpr double MAX_PULSES_PER_LOOP = MAX_SPEED_PPS / CONTROL_LOOP_HZ;

    PIDController* pidController_;  // PID controller for the motor

    /**
     * @brief Constructor for Motor class.
     * @param pin The pin number to which the motor is connected.
     * @param encoderPin The pin number to which the motor encoder is connected.
     */
    Motor(int pin, int encoderPin);

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
     * @brief Map a value from one range to another.
     * @param value The value to be mapped.
     * @param fromMin The minimum value of the input range.
     * @param fromMax The maximum value of the input range.
     * @param toMin The minimum value of the output range.
     * @param toMax The maximum value of the output range.
     * @return The mapped value.
     */
    static double map(double value, double fromMin, double fromMax, double toMin, double toMax) {
        // Calculate the range of the input and output values
        double inputRange = fromMax - fromMin;
        double outputRange = toMax - toMin;

        // Map the value from the input range to the output range
        double mappedValue = ((value - fromMin) * outputRange) / inputRange + toMin;

        return mappedValue;
    }

    // Convert meters per second to -MAX_INT to +MAX_INT
    static double metersToSpeed(double metersPerSecond) {
        return map(metersPerSecond, -MAX_SPEED_MS, MAX_SPEED_MS, -MAX_INT, MAX_INT);
    }

    // Convert max/min int to max/min duty cycle
    static double speedToDutyCycle(double speed) {
        return map(speed, MIN_INT, MAX_INT, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
    }

    // Convert max/min int to meters per second
    static double speedToMS(double speed) {
        return map(speed, -MAX_INT, MAX_INT, -MAX_SPEED_MS, MAX_SPEED_MS);
    }

    // Convert deltaPulses_ to scale of -MAX_INT to +MAX_INT
    static double pulsesToSpeed(double pulses) {
        return map(pulses, -MAX_PULSES_PER_LOOP, MAX_PULSES_PER_LOOP, -MAX_INT, MAX_INT);
    }


    /**
     * @brief Get the speed of the motor commanded.
     * @return The speed of the motor ranging from -MAX_INT to +MAX_INT.
     */
    int16_t getSpeedCmd();

    /**
     * @brief Get the current actual speed of the motor in meters per second.
     * @return The speed of the motor in meters per second.
     */
    double getSpeedMetersPerSecond() {
        // Convert the actualSpeed_ to meters per second
        return actualSpeed_ / PULSES_PER_METER;
    }

    /**
     * @brief Set the speed of the motor.
     * @param speed The speed value to set for the motor.
     *        Negative values indicate reverse, 0 is stopped, and the maximum value is the maximum integer value (-/+32767).
     */
    void setSpeed(int16_t speed);

    /**
     * @brief Set the target speed of the motor.
     * @param targetSpeed The target speed value to set for the motor in meters per second.
     *        Negative values indicate reverse, 0 is stopped.
     */
    void setTargetSpeed(double targetSpeed);

    /**
     * @brief Get the target speed of the motor.
     * @return The target speed of the motor ranging from -MAX_INT to +MAX_INT.
     */
    double getTargetSpeed();

    /**
     * @brief Get the target speed of the motor.
     * @return The target speed of the motor in meters per second.
     */
    double getTargetSpeedMS();

    /**
     * @brief Read the number of pulses counted by the motor.
     */
    void readPulses();

    /**
     * @brief Get the total number of pulses counted by the motor.
     * @return The number of pulses counted by the motor.
     */
    int32_t getPulses();

    /**
     * @brief Get the number of pulses counted by the motor since the last calculatePid call.
     * @return The number of pulses counted by the motor since the last calculatePid call.
     */
    int32_t getDeltaPulses();

    /**
     * @brief Reset the number of pulses counted by the motor.
     */
    void resetDeltaPulses() {
        deltaPulses_ = 0;
    }
    
    /**
     * @brief Get the actual speed of the motor in meters per second.
     * @return The speed of the motor in meters per second.
     */
    double getActualSpeedMS() {
        return actualSpeed_ / PULSES_PER_METER;
    }

    /**
     * @brief Calculate the PID controller output based on the desired speed and the current speed. 
     * Should only be called once per time period (control loop).
     * @return The PID controller output in the range of MIN_INT to MAX_INT.
     */
    int16_t calculatePid();


    /**
     * @brief Convert pulses to radians.
     * @param pulses The number of pulses to convert.
     * @return The equivalent value in radians.
     */
    static double pulsesToRadians(int32_t pulses) {
        // Calculate the conversion factor from pulses to radians
        double conversionFactor = 2.0 * M_PI / PULSES_PER_REV;
        // Convert the pulses to radians
        double radians = pulses * conversionFactor;
        return radians;
    }

    /**
     * @brief Convert radians to pulses.
     * @param radians The number of radians to convert.
     * @return The equivalent value in pulses.
     */
    static int32_t radiansToPulses(double radians) {
        // Calculate the conversion factor from radians to pulses
        double conversionFactor = PULSES_PER_REV / (2.0 * M_PI);
        // Convert the radians to pulses
        int32_t pulses = static_cast<int32_t>(radians * conversionFactor);
        return pulses;
    }

private:
    int pin_;                       // Pin number of the motor
    int encoderPin_;                // Pin number of the motor encoder
    int16_t speed_;                 // current speed command of the motor (in scale of -MAX_INT to +MAX_INT)
    double targetSpeed_;            // desired speed of the motor M/s
    double actualSpeed_;            // current speed of the motor in pulses
    uint slice_;                    // PWM slice number
    uint channel_;                  // PWM channel number
    int32_t pulses_;                // Number of pulses counted by the motor
    int32_t deltaPulses_;           // Number of pulses counted by the motor in the last readPulses call
    int dutyCycle_;                 // PWM duty cycle
};