#pragma once

#include "config.h"
#include <limits>
#include <map>

extern "C" {
    #include "hardware/pwm.h"
    #include "pico/stdlib.h"
}

// Keep the intermediate pulse counts until the next read from the map
static bool gpio_callbacks_configured = false; // Flag to check if the GPIO callbacks are configured

// TODO: Prune this down to use less memory, an array is fast for now
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
    int getPulses();

private:
    int m_pin;          // Pin number of the motor
    int m_encoderPin;   // Pin number of the motor encoder
    int16_t m_speed;    // Current speed of the motor
    uint m_slice;       // PWM slice number
    uint m_channel;     // PWM channel number
    int m_pulses;       // Number of pulses counted by the motor
};