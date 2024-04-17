#ifndef LED_RING_HPP
#define LED_RING_HPP

#include <stdlib.h>

#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "config.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "status.hpp"
#include "ws2812.pio.h"

/**
 * @class LEDRing
 * @brief Represents an LED ring with various animation patterns and color
 * fading capabilities.
 *
 * The LEDRing class provides methods to control an LED ring connected to a
 * specific GPIO pin. It supports rendering different animation patterns, such
 * as snakes, random colors, sparkle, and greyscale. Additionally, it allows
 * fading the LED colors and resetting the animation.
 */
/**
 * @brief Class representing an LED ring.
 */
class LEDRing {
 public:
  /**
   * @brief Constructor for the LEDRing class.
   * @param pin The GPIO pin the LED ring is connected to.
   * @param pio The status machine PIO instance.
   * @param ledCount The number of LEDs in the ring.
   */
  LEDRing(uint8_t pin, PIO pio, uint8_t ledCount);

  /**
   * @brief Starts the LED ring.
   */
  void start();

  /**
   * @brief Renders the LED ring based on the given status.
   * @param status The status to renderStatus.
   */
  void renderStatus(Status status);

  /**
   * @brief Renders the LED ring with the current internal state of LEDs
   */
  void render();

  /**
   * @brief Turns off all LEDs in the LED ring.
   */
  void off() { fill(0, 0, 0); }

  /**
   * @brief Generates a snake pattern animation on the LED ring.
   * @param len The length of the snake.
   * @param t The time variable for the animation.
   */
  void patternSnakes(uint t);

  /**
   * @brief Generates a random pattern animation on the LED ring.
   * @param t The time variable for the animation.
   */
  void patternRandom(uint t);

  /**
   * @brief Generates a sparkle pattern animation on the LED ring.
   * @param t The time variable for the animation.
   */
  void patternSparkle(uint t);

  /**
   * @brief Generates a greyscale pattern animation on the LED ring.
   * @param t The time variable for the animation.
   */
  void patternGreys(uint t);

  /**
   * @brief Fades the LED ring to the specified RGB color over time.
   * @param r The red component of the color.
   * @param g The green component of the color.
   * @param b The blue component of the color.
   * @param t The time variable for the fade animation.
   */
  void fade(uint8_t r, uint8_t g, uint8_t b, uint8_t t);

  /**
   * @brief Fades the red component of the LED ring to the specified value over
   * time.
   * @param r The red component to fade to.
   * @param t The time variable for the fade animation.
   */
  void fadeR(uint8_t r, uint8_t t);

  /**
   * @brief Fades the white component of the LED ring to the specified value
   * over time.
   * @param w The white component to fade to.
   * @param t The time variable for the fade animation.
   */
  void fadeW(uint8_t w, uint8_t t);

  /**
   * @brief Resets the animation of the LED ring.
   */
  void resetAnimation();

  /**
   * @brief Converts RGB color components to a 32-bit unsigned integer.
   * @param r The red component of the color.
   * @param g The green component of the color.
   * @param b The blue component of the color.
   * @return The 32-bit unsigned integer representation of the color.
   */
  static uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b);

  /**
   * @brief Converts RGB color components to a 32-bit unsigned integer with
   * gamma correction.
   * @param r The red component of the color.
   * @param g The green component of the color.
   * @param b The blue component of the color.
   * @return The 32-bit unsigned integer representation of the color with gamma
   * correction.
   */
  static uint32_t urgb_u32_g(uint8_t r, uint8_t g, uint8_t b);

  /**
   * @brief Sets the color of a single LED in the ring.
   * @param pixel_grb The color of the LED in GRB format.
   */
  void putPixel(uint32_t pixel_grb);

  /**
   * @brief Fills the entire LED ring with the specified color.
   * @param r The red component of the color.
   * @param g The green component of the color.
   * @param b The blue component of the color.
   */
  void fill(uint8_t r, uint8_t g, uint8_t b);

  /**
   * @brief Spins the LED ring with the specified color and time.
   * @param r The red component of the color.
   * @param g The green component of the color.
   * @param b The blue component of the color.
   * @param t The time variable for the animation.
   */
  void spin(uint8_t r, uint8_t g, uint8_t b, uint t);

  /**
   * @brief Rolls the LED ring with the specified color and time.
   * @param r The red component of the color.
   * @param g The green component of the color.
   * @param b The blue component of the color.
   * @param t The time variable for the animation.
   */
  void roll(uint8_t r, uint8_t g, uint8_t b, uint t);

 private:
  /**
   * @brief Applies gamma correction to a given value.
   * @param x The value to apply gamma correction to.
   * @return The gamma corrected value.
   */
  static uint8_t gamma_u8(uint8_t x);

  /**
   * @brief Calculates the sine of a given value.
   * @param x The value to calculate the sine of.
   * @return The sine of the value.
   */
  static uint8_t sine(uint8_t x);

  /**
   * @brief Adds two unsigned 8-bit integers with a maximum value of 255.
   * @param x The first value to add.
   * @param y The second value to add.
   * @return The sum of the two values, capped at 255.
   */
  static uint8_t add_u8_max(uint8_t x, uint8_t y);

  uint8_t pin;       // Which GPIO pin the LED ring is connected to
  PIO pio;           // The status machine PIO instance
  uint8_t ledCount;  // How many LEDs are in the ring
  uint8_t t;         // Time variable for animations
  std::vector<std::array<uint8_t, 3>> rgbValues;  // RGB values for each LED
};

// Tables Snagged from
// https://github.com/adafruit/Adafruit_NeoPixel/blob/master/Adafruit_NeoPixel.h

// PROGMEM
static const uint8_t led_ring_sine_table[256] = {
    128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170,
    173, 176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211,
    213, 215, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240,
    241, 243, 244, 245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254,
    254, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251,
    250, 250, 249, 248, 246, 245, 244, 243, 241, 240, 238, 237, 235, 234, 232,
    230, 228, 226, 224, 222, 220, 218, 215, 213, 211, 208, 206, 203, 201, 198,
    196, 193, 190, 188, 185, 182, 179, 176, 173, 170, 167, 165, 162, 158, 155,
    152, 149, 146, 143, 140, 137, 134, 131, 128, 124, 121, 118, 115, 112, 109,
    106, 103, 100, 97,  93,  90,  88,  85,  82,  79,  76,  73,  70,  67,  65,
    62,  59,  57,  54,  52,  49,  47,  44,  42,  40,  37,  35,  33,  31,  29,
    27,  25,  23,  21,  20,  18,  17,  15,  14,  12,  11,  10,  9,   7,   6,
    5,   5,   4,   3,   2,   2,   1,   1,   1,   0,   0,   0,   0,   0,   0,
    0,   1,   1,   1,   2,   2,   3,   4,   5,   5,   6,   7,   9,   10,  11,
    12,  14,  15,  17,  18,  20,  21,  23,  25,  27,  29,  31,  33,  35,  37,
    40,  42,  44,  47,  49,  52,  54,  57,  59,  62,  65,  67,  70,  73,  76,
    79,  82,  85,  88,  90,  93,  97,  100, 103, 106, 109, 112, 115, 118, 121,
    124};

// PROGMEM
static const uint8_t led_ring_gamma_table[256] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,
    1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   3,
    3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,   5,   6,
    6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  10,
    11,  11,  11,  12,  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,
    17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  24,  24,  25,
    25,  26,  27,  27,  28,  29,  29,  30,  31,  31,  32,  33,  34,  34,  35,
    36,  37,  38,  38,  39,  40,  41,  42,  42,  43,  44,  45,  46,  47,  48,
    49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,
    64,  65,  66,  68,  69,  70,  71,  72,  73,  75,  76,  77,  78,  80,  81,
    82,  84,  85,  86,  88,  89,  90,  92,  93,  94,  96,  97,  99,  100, 102,
    103, 105, 106, 108, 109, 111, 112, 114, 115, 117, 119, 120, 122, 124, 125,
    127, 129, 130, 132, 134, 136, 137, 139, 141, 143, 145, 146, 148, 150, 152,
    154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182,
    184, 186, 188, 191, 193, 195, 197, 199, 202, 204, 206, 209, 211, 213, 215,
    218, 220, 223, 225, 227, 230, 232, 235, 237, 240, 242, 245, 247, 250, 252,
    255};

#endif  // LED_RING_HPP