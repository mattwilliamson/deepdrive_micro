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
    static const int MIN_INT = std::numeric_limits<int16_t>::min() + 1; // +1 to avoid overflow
    static const int MAX_INT = std::numeric_limits<int16_t>::max() - 1; // -1 to avoid overflow

    // 1000 us
    static const int DUTY_CYCLE_MIN         = 490;
    // 1500 us
    static const int DUTY_CYCLE_CENTER      = 735;
    // 2000 us
    static const int DUTY_CYCLE_MAX         = 980;
    // surrounding 1500 us
    static constexpr double PULSES_PER_REV = 696; // 34826/50 = 696 | 93156/127 = 734

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
    int map(int value, int fromMin, int fromMax, int toMin, int toMax) {
        // Calculate the range of the input and output values
        int inputRange = fromMax - fromMin;
        int outputRange = toMax - toMin;

        // Map the value from the input range to the output range
        int mappedValue = ((value - fromMin) * outputRange) / inputRange + toMin;

        return mappedValue;
    }

    /**
     * @brief Get the speed of the motor.
     * @return The speed of the motor.
     */
    int16_t getSpeed();

    /**
     * @brief Set the speed of the motor.
     * @param speed The speed value to set for the motor.
     *        Negative values indicate reverse, 0 is stopped, and the maximum value is the maximum integer value.
     */
    void setSpeed(int16_t speed);

    /**
     * @brief Increment the pulse count.
     *
     * The pulses are accumulated in the background as an IRQ.
     * When we read the pulses or change the speed, we read how many were accumulated and reset them.
     */
    void readPulses();

    /**
     * @brief Get the number of pulses counted by the motor.
     * @return The number of pulses counted by the motor.
     */
    int32_t getPulses();

    /**
     * @brief Spin the motor and update the PID controller.
     */
    void spin() {
        // Get the current speed of the motor
        int16_t currentSpeed = getSpeed();
        
        // Update the PID controller with the current speed
        pidController_->calculate(currentSpeed);
    }


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
    int16_t targetSpeed_;           // speed of the motor
    uint slice_;                    // PWM slice number
    uint channel_;                  // PWM channel number
    int32_t pulses_;                // Number of pulses counted by the motor
};