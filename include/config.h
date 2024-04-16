#ifndef CONFIG_H
#define CONFIG_H

#define NDEBUG

// deepdrive_micro/pulses
// deepdrive_micro/cmd

// #define CONTROL_LOOP_HZ 1.0
#define CONTROL_LOOP_HZ 50

// Comment out to disable
// #define WATCHDOG_ENABLED 1

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
#define PIN_LED_STATUS 25
#endif

// END LED STATUS
// ----------------------------------


// ----------------------------------
// START MOTORS
// ----------------------------------

#define MOTOR_COUNT 4

#define PIN_MOTOR_FRONT_LEFT    9
#define PIN_MOTOR_BACK_LEFT     8
#define PIN_MOTOR_FRONT_RIGHT   7
#define PIN_MOTOR_BACK_RIGHT    6

#define IDX_MOTOR_FRONT_LEFT    0
#define IDX_MOTOR_BACK_LEFT     1
#define IDX_MOTOR_FRONT_RIGHT   2
#define IDX_MOTOR_BACK_RIGHT    3

#define MOTOR_JOIN_FRONT_LEFT "wheel_front_left_joint"
#define MOTOR_JOIN_BACK_LEFT "wheel_back_left_joint"
#define MOTOR_JOIN_FRONT_RIGHT "wheel_front_right_joint"
#define MOTOR_JOIN_BACK_RIGHT "wheel_back_right_joint"

// END MOTORS
// ----------------------------------



// ----------------------------------
// START WHEEL ENCODER PULSE COUNTER
// ----------------------------------

#define PIN_ENCODER_FRONT_LEFT    13
#define PIN_ENCODER_BACK_LEFT     12
#define PIN_ENCODER_FRONT_RIGHT   11
#define PIN_ENCODER_BACK_RIGHT    10

// #define GPIO_IRQ_TYPES GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL
#define GPIO_IRQ_TYPES GPIO_IRQ_EDGE_RISE



// END WHEEL ENCODER PULSE COUNTER
// ----------------------------------




// ----------------------------------
// START PID CONTROLLER
// ----------------------------------

// Proportional
#define PID_KP 0.95 // 1.9 starts to oscillate at .1m/s

// Integral
#define PID_KI 0.2 // 0.25 starts to oscillate

// Derivative
#define PID_KD 0.2

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
#define BATTERY_VOLTAGE_REFERENCE 3.3f

// END BATTERY VOLTAGE
// ----------------------------------



// ----------------------------------
// START IMU
// ----------------------------------

// #define IMU_ENABLED 1
#define IMU_I2C_SPEED 400 * 1000
#define IMU_I2C_SDA 4
#define IMU_I2C_SCL 5
#define IMU_I2C {i2c0_hw, false}
#define IMU_ADDRESS 0x68
#define IMU_ADDRESS_MAG 0x0C

#define IMU_FRAME "base_link"

// END IMU
// ----------------------------------



// ----------------------------------
// START DIAGNOSTICS
// ----------------------------------

#define DIAGNOSTIC_FRAME "base_link"
#define DIAGNOSTIC_COUNT 1

// END DIAGNOSTICS
// ----------------------------------

#endif // CONFIG_H