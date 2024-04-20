#include "motor.hpp"

// TODO: Add detection to see if one motor is not moving, which indicates ESC
// failure or reboot

void gpio_callback(uint gpio, uint32_t events) { pulse_count_map[gpio]++; }

Motor::Motor(int pin, int encoderPin) {
  pin_ = pin;
  encoderPin_ = encoderPin;
  targetSpeed_ = 0;
  pulses_ = 0;
  speed_ = 0;
  dutyCycle_ = 0;
  direction_ = 0;

  // https://pidexplained.com/how-to-tune-a-pid-controller/
  pidController_ = new PIDController(-MAX_SPEED_PPS,  // Min
                                     MAX_SPEED_PPS,   // Max
                                     PID_KP,          // Kp
                                     PID_KI,          // Ki
                                     PID_KD           // Kd
  );

  enable();
}

void Motor::start() {
  gpio_init(pin_);
  gpio_set_dir(pin_, GPIO_OUT);
  gpio_set_function(pin_, GPIO_FUNC_PWM);
  slice_ = pwm_gpio_to_slice_num(pin_);
  channel_ = pwm_gpio_to_channel(pin_);
  pwm_set_clkdiv(slice_, 256.0f);
  pwm_set_wrap(slice_, 9804);  // 20 ms
  // pwm_set_enabled(slice_, true);

  // Handle encoder pulses, -1 means no encoder
  if (encoderPin_ != -1) {
    if (!gpio_callbacks_configured) {
      gpio_callbacks_configured = true;
      gpio_set_irq_enabled_with_callback(encoderPin_, GPIO_IRQ_TYPES, true,
                                         &gpio_callback);
    } else {
      gpio_set_irq_enabled(encoderPin_, GPIO_IRQ_TYPES, true);
    }
    gpio_pull_down(encoderPin_);
    pulse_count_map[encoderPin_] = 0;
  }

  // Stop the motor initially
  stop();
}

void Motor::enable() {
  stop();
  pwm_set_enabled(slice_, true);
}

void Motor::disable() {
  stop();
  pwm_set_enabled(slice_, false);
}


void Motor::readPulses() {
  // TODO: Should I keep a couple loops worth and average the pulses?

  // Bitshift left to avoid getting incremented by the ISR while we are trying to get out the value
  pulse_count_map[encoderPin_] << 16;
  uint32_t newPulses = pulse_count_map[encoderPin_] & 0xFFFF0000;
  // clear out the left 16 bits to reset
  pulse_count_map[encoderPin_] &= 0x0000FFFF;
  

  // ------------------------------------------------
  // This version is a known working reference
  // int newPulses = pulse_count_map[encoderPin_];

  // Reset counter
  // pulse_count_map[encoderPin_] = 0;
  // ------------------------------------------------

  // Calculate speed for a whole second
  speed_ = newPulses * CONTROL_LOOP_HZ * direction_;

  if (newPulses > 0 && speed_ != 0) {
    pulses_ += direction_ * newPulses;
  }
}

int16_t Motor::calculatePid() {
  // Get pulses since last loop and extrapolate speed
  readPulses();

  double output = pidController_->calculate(speed_);

  return output;
}

void Motor::setSpeedSignal(Pulses speed) {
  // Set output from the PID controller to pwm out
  speedSignal_ = speed;

  if (speed < 0) {
    direction_ = -1;

  } else if (speed > 0) {
    direction_ = 1;
  } else {
    direction_ = 0;
  }

  // Set the motor PWM duty cycle
  dutyCycle_ = pulsesToDutyCycle(speedSignal_);
  pwm_set_chan_level(slice_, channel_, dutyCycle_);
}

void Motor::setTargetSpeed(Pulses targetSpeed) {
  targetSpeed_ = targetSpeed;
  pidController_->setSetpoint(targetSpeed_);
}

Pulses Motor::getTargetSpeed() { return targetSpeed_; }

void Motor::stop() { setSpeedSignal(0); }

void Motor::updateMotorOutput() {
  // Calculate instantaneous speed and PID controller output
  Pulses pid_output = calculatePid();

  if (getTargetSpeed() == 0) {
    // We're supposed to be stopped, so let reset it all
    stop();
    pidController_->reset();
  } else {
    // Set the motor speed to the output the PID controller calculated
    setSpeedSignal(pid_output);
  }
}