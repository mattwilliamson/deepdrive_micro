#include "motor.h"

Motor::Motor(int pin) {
    m_pin = pin;
    m_speed = 0;
    enable();
}

/**
 * Enables the motor by configuring the necessary PWM settings.
 * This function sets the GPIO function to PWM, sets the clock divider to slow down the clock,
 * sets the wrap time to 9804 (20 ms), and enables the PWM slice.
 */
void Motor::enable() {
    gpio_init(m_pin);
    gpio_set_dir(m_pin, GPIO_OUT);
    gpio_set_function(m_pin, GPIO_FUNC_PWM);
    m_slice = pwm_gpio_to_slice_num(m_pin);
    m_channel = pwm_gpio_to_channel(m_pin);
    pwm_set_clkdiv(m_slice, 256.0f);
    pwm_set_wrap(m_slice, 9804); // 20 ms
    pwm_set_enabled(m_slice, true);

    // Arm the ESC
    // pwm_set_chan_level(m_slice, m_channel, DUTY_CYCLE_MIN);
    // sleep_ms(200);
    // pwm_set_chan_level(m_slice, m_channel, DUTY_CYCLE_MAX);
    // sleep_ms(200);
    
    // Stop the motor initially
    stop();
}

/**
 * Disables the motor by setting the PWM slice associated with the motor pin to disabled.
 */
void Motor::disable() {
    pwm_set_enabled(m_slice, false);
}

/**
 * Sets the speed of the motor.
 *
 * @param speed The speed value to set. Should be within the range of MIN_INT and MAX_INT. 
 * Negative values indicate reverse, 0 is stopped, and the maximum value is the maximum integer value.
 */
void Motor::setSpeed(int16_t speed) {
    // Convert max/min int to max/min duty cycle
    int dutyCycle = map(speed, MIN_INT, MAX_INT, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
    pwm_set_chan_level(m_slice, m_channel, dutyCycle);
}

/**
 * Gets the current speed of the motor.
 *
 * @return The current speed value.
 */
int16_t Motor::getSpeed() {
    return m_speed;
}


/**
 * Stops the motor by setting the speed to 0.
 */
void Motor::stop() {
    setSpeed(0);
}