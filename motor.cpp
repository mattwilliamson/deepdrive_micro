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

    // https://pidexplained.com/how-to-tune-a-pid-controller/
    pidController_ = new PIDController(
        -1.0 * MAX_INT, // Min
        1.0 * MAX_INT, // Max
        PID_KP, // Kp
        PID_KI, // Ki
        PID_KD  // Kd
    );
    
    speed_ = 0;
    dutyCycle_ = 0;
    deltaPulses_ = 0;
    actualSpeed_ = 0;
    
    
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
        gpio_pull_down(encoderPin_);
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

    if (newPulses > 0 && speed_ != 0) {
        // Accumulate the last pulses in case readPulses was read before the next PID controller calculation
        if (speed_ < 0) {
            pulses_ -= newPulses;
            deltaPulses_ -= newPulses;
            actualSpeed_ = -newPulses;
        } else {
            pulses_ += newPulses;
            deltaPulses_ += newPulses;
            actualSpeed_ = newPulses;
        }
    }
}

int32_t Motor::getPulses() {
    // readPulses();
    return pulses_;
}

int32_t Motor::getDeltaPulses() {
    return deltaPulses_;
}

int16_t Motor::calculatePid() {
    // TODO: There's a lot of number conversion back and forth which can be reduced

    // Get instantaneous pulses since last loop
    readPulses();

    // Convert deltaPulses_ to scale of -MAX_INT to +MAX_INT
    double speed = pulsesToSpeed(deltaPulses_);
    double output = pidController_->calculate(speed);

    // Reset for next loop
    deltaPulses_ = 0;

    return output;
}

void Motor::disable() {
    pwm_set_enabled(slice_, false);
}

void Motor::setSpeed(int16_t speed) {
    // Read pulses before we potentially switch directions
    // readPulses();
    speed_ = speed;

    // Convert max/min int to max/min duty cycle
    dutyCycle_ = speedToDutyCycle(speed);
    pwm_set_chan_level(slice_, channel_, dutyCycle_);
}

int16_t Motor::getSpeedCmd() {
    return speed_;
}

void Motor::setTargetSpeed(double targetSpeed) {
    targetSpeed_ = targetSpeed;
    double targetSpeedSignal = metersToSpeed(targetSpeed);
    pidController_->setSetpoint(targetSpeedSignal); // .05 -> 2812, .5 -> 28125
}

double Motor::getTargetSpeed() {
    return targetSpeed_;
}

double Motor::getTargetSpeedMS() {
    return speedToMS(targetSpeed_);
}

void Motor::stop() {
    setSpeed(0);
}