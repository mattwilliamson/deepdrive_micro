/**
 * @file config.h
 * @brief Configuration file for the deepdrive_micro project.
 *
 * This file contains various configuration settings for the deepdrive_micro
 * project. It defines constants and macros related to control loop frequency,
 * LED ring, micro-ros, motors, wheel encoder pulse counter, PID controller,
 * battery voltage, IMU, and diagnostics.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "constants.h"

// #define NDEBUG

// If this is defined, don't actually send motor commands, just simulate them and publish odom
// Good for testing or if we have a different source of odometry
// #define SIMULATE_MOTORS

// If this is defined, don't actually send motor commands. Useful for checking pulses.
// #define DISABLE_MOTORS

// This is for open loop control, where we just set the motor speed and don't use pulse encoders
// TODO: This results in a non-zero speed signal for some reason that loops back into the PID controller
// #define ODOM_OPEN_LOOP

// TODO: Put covariance here

static const int MICRO_METERS = 1e6;
static const int MILLI_METERS = 1e3;
static const int CENTI_METERS = 100.0;

// #define STATUS_LED_ENABLED 1

// Control loop updates the motor speed and pid controllers
// #define CONTROL_LOOP_HZ 4.0
#define CONTROL_LOOP_HZ 30

#define MAIN_LOOP_HZ 10
#define TELEMETRY_LOOP_HZ 2
#define BATTERY_STATE_LOOP_HZ 2

// Run the IMU at a higher rate so the data can be filtered for orientation estimation
#define IMU_LOOP_HZ 100

// Comment out to disable
// #define WATCHDOG_ENABLED

// Reboot the board if we haven't had activity for a while, e.g. no uros agent
#define WATCHDOG_TIMEOUT 30000  // milliseconds

// Time to wait after connecting to agent to consider startup complete
// Pulses will be reset to 0
#define STARTUP_DELAY 1000 // milliseconds

// ----------------------------------
// START LED RING
// ----------------------------------
// GPIO 22

#define LED_RING_ENABLED

#ifdef PICO_DEFAULT_WS2812_PIN
#define LED_RING_PIN PICO_DEFAULT_WS2812_PIN
#else
#define LED_RING_PIN 22
#endif

#define LED_RING_IS_RGBW false
#define LED_RING_NUM_PIXELS 12

#define LED_RING_PIO pio0

#define LED_RING_HZ 100

// END LED RING
// ----------------------------------

// START micro-ros
// ----------------------------------

#define UROS_TIMEOUT 5000
#define UROS_ATTEMPTS 5
// #define UROS_ATTEMPTS 1

// ----------------------------------

// ----------------------------------
// START LED STATUS
// ----------------------------------
// pico w
#if LIB_PICO_CYW43_ARCH
#define PIN_LED_STATUS CYW43_WL_GPIO_LED_PIN
#else
// #define PIN_LED_STATUS 25
#define PIN_LED_STATUS 14
#endif

// END LED STATUS
// ----------------------------------

// ----------------------------------
// START MOTORS
// ----------------------------------

#define CMD_VEL_BUFFER 5

// Time since last cmd_vel message before stopping the robot
#define CMD_VEL_TIMEOUT 10 * NANOSECONDS

// Time since last cmd_vel message before turning off the motors completely
#define CMD_VEL_TIMEOUT_DISABLE 60 * NANOSECONDS

// Time to wait for motors and stay at neutral speed after enabling (3s)
#define MOTOR_NEUTRAL_TIME 2 * NANOSECONDS

// Limit acceleration for each motor
// Meters per second squared
#define MAX_ACCELERATION_LINEAR .75
// #define MAX_ACCELERATION_LINEAR .1

// Radians per second squared
// #define MAX_ACCELERATION_ROTATION .2

#define MOTOR_COUNT 4

#define PIN_MOTOR_FRONT_LEFT 9
#define PIN_MOTOR_BACK_LEFT 8
#define PIN_MOTOR_FRONT_RIGHT 7
#define PIN_MOTOR_BACK_RIGHT 6

#define IDX_MOTOR_FRONT_LEFT 0
#define IDX_MOTOR_BACK_LEFT 1
#define IDX_MOTOR_FRONT_RIGHT 2
#define IDX_MOTOR_BACK_RIGHT 3

#define MOTOR_JOIN_FRONT_LEFT "wheel_front_left_joint"
#define MOTOR_JOIN_BACK_LEFT "wheel_back_left_joint"
#define MOTOR_JOIN_FRONT_RIGHT "wheel_front_right_joint"
#define MOTOR_JOIN_BACK_RIGHT "wheel_back_right_joint"

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

// Motors are stopped in this range
// Duty cycle deadzone 762 - 726 = 36 / 2 = 18
// Middle is (762 + 726) / 2 = 744
// Dead in either direction this amount
#define MOTOR_DUTY_CYCLE_DEADZONE 8

// 1000 us
#define MOTOR_DUTY_CYCLE_STOP      744

// 1000 us - 2000 us -> 490 - 980 duty cycle
// 980 - 490 = 490
#define MOTOR_DUTY_CYCLE_RANGE    490

// 1000 us
#define MOTOR_DUTY_CYCLE_MIN      MOTOR_DUTY_CYCLE_STOP - (MOTOR_DUTY_CYCLE_RANGE / 2)
// 2000 us
#define MOTOR_DUTY_CYCLE_MAX      MOTOR_DUTY_CYCLE_STOP + (MOTOR_DUTY_CYCLE_RANGE / 2)

// How many pulses are output for one revolution of the motor
  // 69579 / 50 = 1,391.58
#define MOTOR_PULSES_PER_REV      1392

// TODO: There is some backlash in the motor, so we need to add some deadband

// Size of the wheels
// 89 * 3.14159 = 280.5 mm per revolution -> .281 meters/rev
#define WHEEL_DIAMETER_MM         89

// Top speed in meters per second that we want to acheive (artificial limit)
#define MOTOR_LIMIT_SPEED_MS      1.0

// Top speed in pulses per second that we want to acheive (artificial limit)
#define MOTOR_LIMIT_SPEED_PPS     (MOTOR_LIMIT_SPEED_MS / METERS_PER_REV * MOTOR_PULSES_PER_REV)


// Measure the pulses per second at a given duty cycle and to figure out what the range is for setting the duty cycle
// Subscribe to the /deepdrive_micro/wheel_speed/out.position[1] and /deepdrive_micro/wheel_speed/out.velocity[1] topics
// #define MOTOR_DUTY_CYCLE_REFERENCE_PWM     744
#define MOTOR_DUTY_CYCLE_REFERENCE_PWM      786

#define MOTOR_REF_SPEED_FRONT_LEFT          2820
#define MOTOR_REF_SPEED_BACK_LEFT           3270
#define MOTOR_REF_SPEED_FRONT_RIGHT         3120
#define MOTOR_REF_SPEED_BACK_RIGHT          2730


// duty cycle = 778
// target speed = .1 m/s
// measured speed = .43 m/s  or 2000 pulses/s


// Max speed that the motor supports (hard limit for mapping speed to pulses)
#define MOTOR_MAX_SPEED_MS        2.0
#define MM_PER_REV                (M_PI * WHEEL_DIAMETER_MM)
#define METERS_PER_REV            (MM_PER_REV / MILLI_METERS)


// Set max speed to about 1.124 m/s (4 revs per second)
// #define MOTOR_MAX_SPEED_PPS       4 * MOTOR_PULSES_PER_REV
// Set max speed to about .281 m/s (1 revs per second)
// #define MOTOR_MAX_SPEED_PPS       MOTOR_PULSES_PER_REV
#define MOTOR_MAX_SPEED_PPS       (MOTOR_MAX_SPEED_MS / METERS_PER_REV * MOTOR_PULSES_PER_REV)

// Don't accept values this high since they are impossible
#define MOTOR_MAX_SPEED_FILTER    (5 * MOTOR_PULSES_PER_REV)

// How far apart the wheels are in mm from left to right innermost edge
#define WHEEL_BASE_MM             240

// Multiplier to adjust the wheel base to compensate for slippage
// #define WHEEL_BASE_COEFFICIENT    1.0
#define WHEEL_BASE_COEFFICIENT    2.5

// END MOTORS
// ----------------------------------


// ----------------------------------
// START WHEEL ENCODER PULSE COUNTER
// ----------------------------------

// If we have noisy encoders, we might get pulses when we aren't actually moving. This will ignore them if speed = 0;
// #define WHEEL_ENCODER_IGNORE_STOPPED
#define PIN_ENCODER_FRONT_LEFT 13
#define PIN_ENCODER_BACK_LEFT 12
#define PIN_ENCODER_FRONT_RIGHT 11
#define PIN_ENCODER_BACK_RIGHT 10

#define GPIO_IRQ_TYPES GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL
// #define GPIO_IRQ_TYPES GPIO_IRQ_EDGE_RISE

// Pulses less than this will be ignore as noise
#define ENCODER_NOISE_THRESHOLD 3

// How many samples to keep and average out
// Might affect PID_KP macro below
#define ENCODER_PULSE_BUFFER 20

// END WHEEL ENCODER PULSE COUNTER
// ----------------------------------


// ----------------------------------
// START PID CONTROLLER
// ----------------------------------

// // Proportional
// #define PID_KP 0.95  // 1.9 starts to oscillate at .1m/s

// // Integral
// #define PID_KI 0.1  // 0.25 starts to oscillate

// // Derivative
// #define PID_KD 0.2

// Proportional
// #define PID_KP .001
#define PID_KP 0.75
// #define PID_KP 0.019

// Integral
// #define PID_KI 0
#define PID_KI 0.5
// #define PID_KI 0.0015
// #define PID_KI 0.002

// Derivative
#define PID_KD 0
// #define PID_KD 0.05
// #define PID_KD 0.000



// END PID CONTROLLER
// ----------------------------------

// ----------------------------------
// START BATTERY VOLTAGE
// ----------------------------------

// Battery voltage divider pin
#define PIN_BATTERY_VOLTAGE 26

// Battery voltage divider input number (GPIO26 ADC0)
#define PIN_BATTERY_VOLTAGE_INPUT 0

// Battery voltage reference
// ADC raw value = 2356, multimeter raw value = 11.08
// ADC_REF = 3.277, Voltage Divider = 1.902, ADC Calculated =1.8839550018310547
#define BATTERY_VOLTAGE_REFERENCE 3.277f
#define BATTERY_VOLTAGE_CONVERSION 11.07f / 1.8839550018310547f
#define BATTERY_CELLS 4
#define BATTERY_CAPACITY 5200  // mAh

// END BATTERY VOLTAGE
// ----------------------------------

// ----------------------------------
// START IMU
// ----------------------------------

#define IMU_PUBLISH
#define IMU_ENABLED
#define IMU_I2C_SPEED 400 * 1000
#define IMU_I2C_SDA 4
#define IMU_I2C_SCL 5
#define IMU_I2C { i2c0_hw, false }
#define IMU_ADDRESS 0x68
#define IMU_ADDRESS_MAG 0x0C

#define IMU_FRAME "imu_link"
#define BATTERY_FRAME "base_link"

// END IMU
// ----------------------------------

// ----------------------------------
// START DIAGNOSTICS
// ----------------------------------

#define DIAGNOSTIC_FRAME "base_link"
#define DIAGNOSTIC_COUNT 1
#define DIAGNOSTIC_ROWS 4
#define DIAGNOSTIC_MESSAGE_LEN 50
#define DIAGNOSTIC_NUMBER_LEN 20

// END DIAGNOSTICS
// ----------------------------------


// ----------------------------------
// START BUZZER
// ----------------------------------

// #define BUZZER_ENABLED
#define PIN_BUZZER 17

// Percent to start warning beeps at
#define BUZZER_BATTERY_WARN 20.0

// Percent to start error beeps at
#define BUZZER_BATTERY_ERROR 10.0

#define BUZZER_WARN_INTERVAL 30 * NANOSECONDS  // 1 second
#define BUZZER_ERROR_INTERVAL 10 * NANOSECONDS  // 1 second

// ----------------------------------



// ----------------------------------
// START SONAR
// ----------------------------------

// #define SONAR_ENABLED

#define SONAR_PIO pio1

#define SONAR_SENSORS 2

#define SONAR_TRIGGER_PIN_FRONT     21
#define SONAR_ECHO_PIN_FRONT        20
#define SONAR_TRIGGER_PIN_BACK      19
#define SONAR_ECHO_PIN_BACK         18

// Since a small object will cause a bounce, only use closes ranged objects for navigation
// #define SONAR_MAX_DISTANCE 4.0f  // meters
#define SONAR_MAX_DISTANCE 2.0f  // meters
#define SONAR_MIN_DISTANCE 0.02f  // meters
#define SONAR_FOV 15  // degrees
#define SONAR_PUBLISH_RATE 10 // Hz
#define SONAR_FRAME_FRONT "sonar_front_link" // Need to add separate frame
#define SONAR_FRAME_BACK "sonar_back_link" // Need to add separate frame
#define SONAR_TOPIC_FRONT "~/sonar/front"
#define SONAR_TOPIC_BACK "~/sonar/back"
#define SONAR_LASER_RAYS 30

// PIO FIFOs are only four words (of 32 bits)
#define SONAR_SAMPLES 4


#endif  // CONFIG_H