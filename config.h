#pragma once

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

