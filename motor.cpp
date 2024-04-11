#include "motor.h"

// TODO: Add detection to see if one motor is not moving, which indicates ESC failure or reboot

void gpio_callback(uint gpio, uint32_t events) {
    pulse_count_map[gpio]++;
}

Motor::Motor(int pin, int encoderPin) {
    pin_ = pin;
    encoderPin_ = encoderPin;
    targetSpeed_ = 0;
    pulses_ = 0;
    pidController_ = new PIDController();
    // pidController_.calculate
    enable();
}

void Motor::enable() {
    gpio_init(pin_);
    gpio_set_dir(pin_, GPIO_OUT);
    gpio_set_function(pin_, GPIO_FUNC_PWM);
    slice_ = pwm_gpio_to_slice_num(pin_);
    channel_ = pwm_gpio_to_channel(pin_);
    pwm_set_clkdiv(slice_, 256.0f);
    pwm_set_wrap(slice_, 9804); // 20 ms
    pwm_set_enabled(slice_, true);

    // Handle encoder pulses, -1 means no encoder
    if (encoderPin_ != -1) {
        if (!gpio_callbacks_configured) {
            gpio_callbacks_configured = true;
            gpio_set_irq_enabled_with_callback(encoderPin_, GPIO_IRQ_TYPES, true, &gpio_callback);
        } else {
            gpio_set_irq_enabled(encoderPin_, GPIO_IRQ_TYPES, true);
        }
        pulse_count_map[encoderPin_] = 0;
    }
    // Arm the ESC
    // pwm_set_chan_level(slice_, channel_, DUTY_CYCLE_MIN);
    // sleep_ms(200);
    // pwm_set_chan_level(slice_, channel_, DUTY_CYCLE_MAX);
    // sleep_ms(200);
    
    // Stop the motor initially
    stop();
}

void Motor::readPulses() {
    // TODO: See if we can make this atomic somehow to avoid missing pulses
    int newPulses = pulse_count_map[encoderPin_];
    pulse_count_map[encoderPin_] = 0;

    if (targetSpeed_ < 0) {
        pulses_ -= newPulses;
    } else {
        pulses_ += newPulses;
    }
}

int32_t Motor::getPulses() {
    readPulses();
    return pulses_;
}

void Motor::disable() {
    pwm_set_enabled(slice_, false);
}

void Motor::setSpeed(int16_t speed) {
    // Read pulses before we potentially switch directions
    readPulses();

    // Convert max/min int to max/min duty cycle
    int dutyCycle = map(speed, MIN_INT, MAX_INT, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
    pwm_set_chan_level(slice_, channel_, dutyCycle);
}

int16_t Motor::getSpeed() {
    return targetSpeed_;
}

void Motor::stop() {
    setSpeed(0);
}