#include "motor.hpp"

// TODO: Add detection to see if one motor is not moving, which indicates ESC
// failure or reboot

void gpio_callback(uint gpio, uint32_t events) {
  pulse_count_map[gpio]++;
}

Motor::Motor(Side side, int pin, int encoderPin) {
  side_ = side;
  pin_ = pin;
  encoderPin_ = encoderPin;
  targetSpeed_ = 0;
  pulses_ = 0;
  speed_ = 0;
  dutyCycle_ = 0;
  direction_ = 0;
  pulses_loop_ = 0;
  speedSignal_ = 0;
  lastRead_ = time_us_64();

  // https://pidexplained.com/how-to-tune-a-pid-controller/
  pidController_ = new PIDController(-MAX_SPEED_PPS,  // Min
                                     MAX_SPEED_PPS,   // Max
                                     PID_KP,          // Kp
                                     PID_KI,          // Ki
                                     PID_KD           // Kd
  );
  start();
  setTargetSpeed(0);
}

void Motor::start() {
#ifndef DISABLE_MOTORS
  gpio_init(pin_);
  gpio_set_dir(pin_, GPIO_OUT);
  gpio_set_function(pin_, GPIO_FUNC_PWM);
#endif
  slice_ = pwm_gpio_to_slice_num(pin_);
  channel_ = pwm_gpio_to_channel(pin_);

  pwm_set_clkdiv(slice_, 256.0f);
  pwm_set_wrap(slice_, 9804);  // 20 ms

  // Handle encoder pulses, -1 means no encoder
  if (encoderPin_ != -1) {
    if (!gpio_callbacks_configured) {
      gpio_callbacks_configured = true;
      gpio_set_irq_enabled_with_callback(encoderPin_, GPIO_IRQ_TYPES, true, &gpio_callback);
    } else {
      gpio_set_irq_enabled(encoderPin_, GPIO_IRQ_TYPES, true);
    }

    // gpio_pull_up(encoderPin_);
    gpio_pull_down(encoderPin_);
    pulse_count_map[encoderPin_] = 0;
  }

  // enable();
  stop();
  disable();
}

void Motor::enable() {
  if (enabled_) {
    return;
  }

  enabled_ = true;
  enabled_time_ = rmw_uros_epoch_nanos();

#ifndef SIMULATE_MOTORS
  pwm_set_enabled(slice_, true);
#endif

  stop();
}

void Motor::disable() {
  stop();
  enabled_ = false;

#ifndef SIMULATE_MOTORS
  pwm_set_enabled(slice_, false);
#endif
}

void Motor::readPulses() {
  // Bitshift left to avoid getting incremented by the ISR while we are trying to get out the value
  pulse_count_map[encoderPin_] <<= 16;
  uint32_t newPulses = pulse_count_map[encoderPin_];
  newPulses &= 0xFFFF0000;
  newPulses >>= 16;

  // clear out the left 16 bits to reset
  pulse_count_map[encoderPin_] &= 0x0000FFFF;

  // Impossibly high. Lets just max it out so the PID controller doesn't go crazy
  if (newPulses > MOTOR_MAX_SPEED_FILTER) {
    // printf("Speed too high: %d\n", newPulses);
    newPulses = MOTOR_MAX_SPEED_FILTER;
    // return;
  }

#ifdef SIMULATE_MOTORS
  // For testing. We are not actually using the motors.
  static int16_t lastSpeedSignal = 0;
  // Simulate the pulses with a little noise
  int noise = rand() % 2;
  // How much weight to put on the previous value
  static const int16_t prevWeight = 5;
  int16_t weightedPulses = ((prevWeight * lastSpeedSignal) + speedSignal_) / (prevWeight + 1);
  newPulses = (weightedPulses / CONTROL_LOOP_HZ) + noise;
  lastSpeedSignal = speedSignal_;
#endif

  // Update total pulses
  // TODO: We might need to update direction after we have changed speed so we don't miss pulses

  if (newPulses > ENCODER_NOISE_THRESHOLD) {
    pulses_ += newPulses * direction_;
  } else {
    // Ignore noise
    newPulses = 0;
  }

  // // Reject any speeds that are impossible to compensate for encoder noise
  // if (newPulses > (MAX_SPEED_PPS / CONTROL_LOOP_HZ)) {
  //   printf("Speed too high: %d\n", newPulses);
  //   return;
  // }

  // Average out the last few pulses we've read
  pulseBuffer_.push((Pulses)newPulses * direction_);
  pulses_loop_ = pulseBuffer_.average();

  Pulses new_speed = pulses_loop_ * CONTROL_LOOP_HZ;

  // Calculate speed for a whole second
  speed_ = new_speed;
}

int16_t Motor::calculatePid() {
  // TODO: Use time delta to feed into the PID controller

  uint64_t timeDelta = time_us_64() - lastRead_;
  double elapsed = (double)timeDelta / 1e6;

  double output = pidController_->calculate(elapsed, speed_);

#ifdef ODOM_OPEN_LOOP
  // Open loop control, just set the speed and ignore pulses.
  return targetSpeed_;
#endif

  lastRead_ = time_us_64();

  return output;
}

void Motor::setSpeedSignal(Pulses speed) {
  // Set output from the PID controller to pwm out
  speedSignal_ = speed;

  if (speed < 0) {
    // Backward
    direction_ = -1;
  } else if (speed > 0) {
    // Forward
    direction_ = 1;
  } else {
    // Stopped
    // TODO: Set direction to 0 when we are finished stopping
#ifdef WHEEL_ENCODER_IGNORE_STOPPED
    direction_ = 0;
#else
    direction_ = 1;
#endif
  }

  // Get the motor PWM duty cycle
  dutyCycle_ = pulsesToDutyCycle(speedSignal_);

  // #ifndef DISABLE_MOTORS
  // Set the PWM duty cycle
  pwm_set_chan_level(slice_, channel_, dutyCycle_);
  // #endif
}

void Motor::setTargetSpeed(Pulses targetSpeed) {
  targetSpeed_ = targetSpeed;
  pidController_->setSetpoint(targetSpeed_);
}

Pulses Motor::getTargetSpeed() {
  return targetSpeed_;
}

Meters Motor::getTargetSpeedMeters() {
  return pulsesToMeters(targetSpeed_);
}

void Motor::stop() { 
  setTargetSpeed(0);
  setSpeedSignal(0); 
}

void Motor::updateMotorOutput() {
  // Get pulses since last loop and extrapolate speed
  readPulses();

  if (getTargetSpeed() == 0) {
    // We're supposed to be stopped, so let reset it all
    stop();
    pidController_->reset();
    lastRead_ = time_us_64();
    return;
  }

  // Make sure the motor has been idle long enough for the ESC to start up
  if (enabled_ && rmw_uros_epoch_nanos() - enabled_time_ > MOTOR_NEUTRAL_TIME) {
    // Calculate instantaneous speed and PID controller output
    Pulses pid_output = calculatePid();
    setSpeedSignal(pid_output);
  } else {
    lastRead_ = time_us_64();
    setSpeedSignal(0);
  }
}