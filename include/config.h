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

// #define NDEBUG

// If this is defined, don't actually send motor commands, just simulate them and publish odom
// Good for testing or if we have a different source of odometry
// #define ODOM_SIMULATE

// This is for open loop control, where we just set the motor speed and don't use encoders
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

// ----------------------------------
// START LED RING
// ----------------------------------
// GPIO 22

#define LED_RING_ENABLED 1

#ifdef PICO_DEFAULT_WS2812_PIN
#define LED_RING_PIN PICO_DEFAULT_WS2812_PIN
#else
#define LED_RING_PIN 22
#endif

#define LED_RING_IS_RGBW false
#define LED_RING_NUM_PIXELS 12

#define LED_RING_PIO pio0

#define LED_RING_HZ 10

// END LED RING
// ----------------------------------

// START micro-ros
// ----------------------------------

#define UROS_TIMEOUT 1000
#define UROS_ATTEMPTS 120
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

// Time since last cmd_vel message before stopping the robot
#define CMD_VEL_TIMEOUT 1 * NANOSECONDS

// Meters per second squared
#define MAX_ACCELERATION_LINEAR .1

// Radians per second squared
#define MAX_ACCELERATION_ROTATION .2

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
// #define PID_KP (0.02 * CONTROL_LOOP_HZ)
#define PID_KP (0.01 * CONTROL_LOOP_HZ)
// #define PID_KP 0.019 * CONTROL_LOOP_HZ

// Integral
#define PID_KI (0.002 * CONTROL_LOOP_HZ)
// #define PID_KI (0.0015 * CONTROL_LOOP_HZ)
// #define PID_KI (0.002 * CONTROL_LOOP_HZ)

// Derivative
#define PID_KD (0.01 * CONTROL_LOOP_HZ)
// #define PID_KD (0.000 * CONTROL_LOOP_HZ)



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

#define IMU_ENABLED 1
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

#define BUZZER_ENABLED 1
#define BUZZER_PIN 17

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

#define SONAR_ENABLED 1
// TODO: Front sensor and back sensor
#define SONAR_TRIGGER_PIN_FRONT 21
#define SONAR_ECHO_PIN_FRONT 20
#define SONAR_TRIGGER_PIN_BACK 19
#define SONAR_ECHO_PIN_BACK 18

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

// PIO FIFOs are only four words (of 32 bits)
#define SONAR_SAMPLES 4


#endif  // CONFIG_H