#ifndef LED_STATUS_HPP
#define LED_STATUS_HPP

#include <stdint.h>
#include <stdio.h>

#include "config.h"
#include "pico/stdlib.h"

// On the Pico W, the LED is connected to the wifi module
#if LIB_PICO_CYW43_ARCH
#include "pico/cyw43_arch.h"
#endif

/**
 * @brief Initializes the LED status module.
 */
void led_status_init();

/**
 * @brief Sets the LED status.
 *
 * @param status The status to set (true for ON, false for OFF).
 */
void led_status_set(bool status);

/**
 * @brief Blinks the LED for a specified number of times with specified on and
 * off times synchronously.
 *
 * @param count The number of times to blink the LED.
 * @param on_time_ms The duration of the LED ON state in milliseconds.
 * @param off_time_ms The duration of the LED OFF state in milliseconds.
 */
void led_status_blink(uint count, uint32_t on_time_ms, uint32_t off_time_ms);

#endif  // LED_STATUS_HPP