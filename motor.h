#pragma once

#include <limits>

extern "C" {
    #include "hardware/pwm.h"
    #include "pico/stdlib.h"
}

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
    // Max speed backward
    static const int MIN_INT = std::numeric_limits<int16_t>::min() + 1; // +1 to avoid overflow
    // Max speed forward
    static const int MAX_INT = std::numeric_limits<int16_t>::max() - 1; // -1 to avoid overflow

    // 1000 us
    static const int DUTY_CYCLE_MIN         = 490;
    // 1500 us
    static const int DUTY_CYCLE_CENTER      = 735;
    // 2000 us
    static const int DUTY_CYCLE_MAX         = 980;
    // surrounding 1500 us
    static const int DUTY_CYCLE_DEADBAND    = 100;

    /**
     * @brief Constructor for Motor class.
     * @param pin The pin number to which the motor is connected.
     */
    Motor(int pin);

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
     * @brief Get the current speed of the motor.
     * @return The current speed of the motor.
     */
    int16_t getSpeed();

    /**
     * @brief Set the current speed of the motor.
     * @param speed The speed value to set for the motor.
     *        Negative values indicate reverse, 0 is stopped, and the maximum value is the maximum integer value.
     */
    void setSpeed(int16_t speed);

private:
    int m_pin;      // Pin number of the motor
    int16_t m_speed;    // Current speed of the motor
    uint m_slice;   // PWM slice number
    uint m_channel; // PWM channel number
};