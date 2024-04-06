#pragma once

// deepdrive_micro/pulses
// deepdrive_micro/cmd

// Comment out to disable
// #define WATCHDOG_ENABLED 1

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

#define LED_RING_DELAY_MS 10
// #define LED_RING_DELAY_MS 100

// END LED RING
// ----------------------------------



// ----------------------------------
// START MOTORS
// ----------------------------------

#define MOTOR_COUNT 4
#define PIN_MOTOR_FRONT_LEFT    9
#define PIN_MOTOR_FRONT_RIGHT   7
#define PIN_MOTOR_BACK_LEFT     8
#define PIN_MOTOR_BACK_RIGHT    6

// END MOTORS
// ----------------------------------



// ----------------------------------
// START PULSE COUNTER
// ----------------------------------

#define PIN_ENCODER_FRONT_LEFT    13
#define PIN_ENCODER_FRONT_RIGHT   11
#define PIN_ENCODER_BACK_LEFT     12
#define PIN_ENCODER_BACK_RIGHT    10

#define GPIO_IRQ_TYPES GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL

// END PULSE COUNTER
// ----------------------------------
