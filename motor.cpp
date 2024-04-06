#include "motor.h"

void gpio_callback(uint gpio, uint32_t events) {
    pulse_count_map[gpio]++;
}

Motor::Motor(int pin, int encoderPin) {
    m_pin = pin;
    m_encoderPin = encoderPin;
    m_speed = 0;
    m_pulses = 0;
    enable();
}

void Motor::enable() {
    gpio_init(m_pin);
    gpio_set_dir(m_pin, GPIO_OUT);
    gpio_set_function(m_pin, GPIO_FUNC_PWM);
    m_slice = pwm_gpio_to_slice_num(m_pin);
    m_channel = pwm_gpio_to_channel(m_pin);
    pwm_set_clkdiv(m_slice, 256.0f);
    pwm_set_wrap(m_slice, 9804); // 20 ms
    pwm_set_enabled(m_slice, true);

    // Handle encoder pulses, -1 means no encoder
    if (m_encoderPin != -1) {
        if (!gpio_callbacks_configured) {
            gpio_callbacks_configured = true;
            gpio_set_irq_enabled_with_callback(m_pin, GPIO_IRQ_TYPES, true, &gpio_callback);
        } else {
            gpio_set_irq_enabled(m_pin, GPIO_IRQ_TYPES, true);
        }
        pulse_count_map[m_encoderPin] = 0;
    }
    // Arm the ESC
    // pwm_set_chan_level(m_slice, m_channel, DUTY_CYCLE_MIN);
    // sleep_ms(200);
    // pwm_set_chan_level(m_slice, m_channel, DUTY_CYCLE_MAX);
    // sleep_ms(200);
    
    // Stop the motor initially
    stop();
}

void Motor::readPulses() {
    // TODO: See if we can make this atomic somehow to avoid missing pulses
    int newPulses = pulse_count_map[m_encoderPin];
    pulse_count_map[m_encoderPin];

    if (m_speed < 0) {
        m_pulses -= newPulses;
    } else {
        m_pulses += newPulses;
    }
}

// WARNING: If we switch directions before
int Motor::getPulses() {
    readPulses();
    return m_pulses;
}

void Motor::disable() {
    pwm_set_enabled(m_slice, false);
}

void Motor::setSpeed(int16_t speed) {
    // Read pulses before we potentially switch directions
    readPulses();

    // Convert max/min int to max/min duty cycle
    int dutyCycle = map(speed, MIN_INT, MAX_INT, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
    pwm_set_chan_level(m_slice, m_channel, dutyCycle);
}

int16_t Motor::getSpeed() {
    return m_speed;
}

void Motor::stop() {
    setSpeed(0);
}