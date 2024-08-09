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

#define SerialDebug Serial1


// Ping the agent at startup or reboot
#define UROS_TIMEOUT_STARTUP 1000 /**< Timeout value for UROS communication (ms). */
#define UROS_ATTEMPTS_STARTUP 10 /**< Number of attempts for UROS communication. */

// Ping the agent periodically
#define UROS_TIMEOUT_PERIODIC 50 /**< Timeout value for UROS communication (ms). */
#define MESSAGE_RECEIVE_TIMEOUT 60000 /**< Start pinging the microros-agent if we haven't had a message for a while */
#define AGENT_PING_ATTEMPTS_REBOOT 10 /**< Reboot the board if we can't reach the agent for 200 * 100 milliseconds (20s) */

#define BOARD_DELAY 10000 /**< Delay in microseconds for board initialization. */
#define SERIAL_BAUD_RATE 115200 /**< Baud rate for serial communication. */


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


#define MAIN_LOOP_HZ 10
#define TELEMETRY_LOOP_HZ 2
#define BATTERY_STATE_LOOP_HZ 2

// Comment out to disable
// #define WATCHDOG_ENABLED

// Reboot the board if we haven't had activity for a while, e.g. no uros agent
#define WATCHDOG_TIMEOUT 30000  // milliseconds

// Time to wait after connecting to agent to consider startup complete
// Pulses will be reset to 0
#define STARTUP_DELAY 1000 // milliseconds


// ----------------------------------
// START IMU
// ----------------------------------

#define IMU_ENABLED

#define IMU_I2C_SPEED 400 * 1000
#define IMU_WIRE_PORT Wire
// #define IMU_I2C_ADDRESS 0x68
#define IMU_AD0_VAL 0
// #define IMU_I2C_ADDRESS_MAG 0x0C
#define IMU_FRAME_ID "imu_base_link"
#define IMU_PUBLISH_RATE 30  // Hz
#define IMU_MAX_RETRY_COUNT 5
#define IMU_RETRY_DELAY_MS 1000
#define IMU_MAX_COVARIANCE 0.1
#define IMU_MIN_COVARIANCE 0.1
#define IMU_TOPIC_NAME "~/imu/data"
#define IMU_BIAS_SAVE_INTERVAL_MS 2 * 60 * 1000 // Save biases after 2 minutes
// #define IMU_BIAS_SAVE_INTERVAL_MS 20 * 1000 // Save biases after 2 minutes

// TODO: Declination?

// #define IMU_ENABLE_EEPROM

// END IMU


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
#define BATTERY_FRAME "base_link"

// END BATTERY VOLTAGE
// ----------------------------------


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

#define SONAR_ENABLED

#define SONAR_SENSORS 2

#define SONAR_TRIGGER_PIN_RIGHT     21
#define SONAR_ECHO_PIN_RIGHT        20
#define SONAR_TRIGGER_PIN_LEFT      19
#define SONAR_ECHO_PIN_LEFT         18

// Since a small object will cause a bounce, only use closes ranged objects for navigation
// #define SONAR_MAX_DISTANCE 4.0f  // meters
#define SONAR_MAX_DISTANCE 2.0f  // meters
#define SONAR_MIN_DISTANCE 0.02f  // meters
#define SONAR_FOV 15  // degrees
#define SONAR_PUBLISH_RATE 10 // 10 // Hz
#define SONAR_PING_RATE 10 // Hz
#define SONAR_FRAME_LEFT "sonar_left_link" // Need to add separate frame
#define SONAR_FRAME_RIGHT "sonar_right_link" // Need to add separate frame
#define SONAR_TOPIC_RANGE_LEFT "~/sonar/left/range"
#define SONAR_TOPIC_RANGE_RIGHT "~/sonar/right/range"
#define SONAR_TOPIC_SCAN_LEFT "~/sonar/left/scan"
#define SONAR_TOPIC_SCAN_RIGHT "~/sonar/right/scan"
#define SONAR_LASER_RAYS 30

// PIO FIFOs are only four words (of 32 bits)
#define SONAR_SAMPLES 4

// ----------------------------------


#endif  // CONFIG_H