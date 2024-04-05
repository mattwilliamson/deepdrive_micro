#pragma once

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
#define WS2812_PIN PICO_DEFAULT_WS2812_PIN
#else
#define WS2812_PIN 22
#endif


#define LED_RING_IS_RGBW false
#define LED_RING_NUM_PIXELS 12

// END LED RING
// ----------------------------------

