#include "led_ring.hpp"

#include "status.hpp"

LEDRing::LEDRing(uint8_t pin, PIO pio, uint8_t ledCount)
    : pin(pin), pio(pio), ledCount(ledCount) {
  rgbValues.resize(ledCount, {0, 0, 0});
  resetAnimation();
}

void LEDRing::start() {
  t = 0;
  stdio_init_all();
  // printf("WS2812 Smoke Test, using pin %d", pin);
  int sm = pio_claim_unused_sm(pio, false);
  uint offset = pio_add_program(pio, &ws2812_program);
  ws2812_program_init(pio, sm, offset, pin, 800000, LED_RING_IS_RGBW);
}

void LEDRing::resetAnimation() {
  t = 0;
  // loop over rgb values and set them to 0
  for (uint i = 0; i < ledCount; ++i) {
    rgbValues[i] = {0, 0, 0};
  }
}

inline uint32_t LEDRing::urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

inline uint32_t LEDRing::urgb_u32_g(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)(gamma_u8(r)) << 8) | ((uint32_t)(gamma_u8(g)) << 16) |
         (uint32_t)(gamma_u8(b));
}

void LEDRing::render() {
  // Loop through all the LEDs and push to the hardware
  for (uint i = 0; i < ledCount; ++i) {
    putPixel(urgb_u32_g(rgbValues[i][0], rgbValues[i][1], rgbValues[i][2]));
  }
}

void LEDRing::renderStatus(Status status) {
  switch (status) {
    case Status::Connecting:
      // fadeW(0x66, t);
      spin(0x33, 0x66, 0x99, t / 4);
      // roll(0x33, 0x66, 0x99, t);
      break;
    case Status::Connected:
      fill(0x00, 0x99, 0x00);
      break;
    case Status::Error:
      fadeR(0x99, t);
      break;
    case Status::Warning:
      roll(0x99, 0x66, 0x00, t);
      break;
    case Status::Active:
      fill(0x66, 0x66, 0x66);
      break;
    case Status::Rebooted:
      fill(0x99, 0x66, 0x00);
      break;
    case Status::Init:
      fadeW(0x99, t);
      break;
    default:
      fill(0x33, 0x33, 0x33);
      break;
  }

  if (++t == UINT8_MAX) {
    t = 0;
  }
}

void LEDRing::patternSnakes(uint t) {
  for (uint i = 0; i < ledCount; ++i) {
    uint x = (i + (t >> 1)) % 64;
    if (x < 10)
      putPixel(urgb_u32(0xff, 0, 0));
    else if (x >= 15 && x < 25)
      putPixel(urgb_u32(0, 0xff, 0));
    else if (x >= 30 && x < 40)
      putPixel(urgb_u32(0, 0, 0xff));
    else
      putPixel(0);
  }
}

void LEDRing::patternRandom(uint t) {
  if (t % 8) return;
  for (int i = 0; i < ledCount; ++i) putPixel(rand());
}

void LEDRing::patternSparkle(uint t) {
  if (t % 8) return;
  for (int i = 0; i < ledCount; ++i) putPixel(rand() % 16 ? 0 : 0xffffffff);
}

void LEDRing::patternGreys(uint t) {
  int max = 100;  // let's not draw too much current!
  t %= max;
  for (int i = 0; i < ledCount; ++i) {
    putPixel(t * 0x10101);
    if (++t >= max) t = 0;
  }
}

void LEDRing::fade(uint8_t r, uint8_t g, uint8_t b, uint8_t t) {
  uint8_t r2 = (uint8_t)((uint16_t)sine(t) * r / UINT8_MAX);
  uint8_t g2 = (uint8_t)((uint16_t)sine(t) * g / UINT8_MAX);
  uint8_t b2 = (uint8_t)((uint16_t)sine(t) * b / UINT8_MAX);

  fill(r2, g2, b2);
}

void LEDRing::fadeR(uint8_t r, uint8_t t) {
  uint8_t r2 = (uint8_t)((uint16_t)sine(t) * r / UINT8_MAX);
  fill(r2, 0, 0);
}

void LEDRing::fadeW(uint8_t w, uint8_t t) {
  uint8_t w2 = (uint8_t)((uint16_t)sine(t) * w / UINT8_MAX);
  fill(w2, w2, w2);
}

void LEDRing::putPixel(uint32_t pixel_grb) {
  pio_sm_put_blocking(pio, 0, pixel_grb << 8u);
}

void LEDRing::fill(uint8_t r, uint8_t g, uint8_t b) {
  for (uint i = 0; i < ledCount; ++i) {
    putPixel(urgb_u32(gamma_u8(r), gamma_u8(g), gamma_u8(b)));
  }
}

void LEDRing::spin(uint8_t r, uint8_t g, uint8_t b, uint t) {
  for (uint i = 0; i < ledCount; ++i) {
    if (i == t % ledCount) {
      putPixel(urgb_u32(gamma_u8(r), gamma_u8(g), gamma_u8(b)));
    } else {
      putPixel(0);
    }
  }
}

void LEDRing::roll(uint8_t r, uint8_t g, uint8_t b, uint t) {
  // Loop over all the LEDs and set the color
  // Make half white and half the color specified and rotate based on t
  // set colors in rgbValues and then render them
  // Offset by ledCount to avoid overflowing
  uint8_t m = ledCount + (t % ledCount);
  for (uint i = 0; i < ledCount; ++i) {
    if (i + ledCount < m - ledCount / 2 || i + ledCount > m + ledCount / 2) {
      rgbValues[i] = {r, g, b};
    } else {
      rgbValues[i] = {0, 0, 0};
    }
  }

  // Draw them
  render();
}

inline uint8_t LEDRing::gamma_u8(uint8_t x) { return led_ring_gamma_table[x]; }

inline uint8_t LEDRing::sine(uint8_t x) { return led_ring_sine_table[x]; }

inline uint8_t LEDRing::add_u8_max(uint8_t x, uint8_t y) {
  if (x + y < UINT8_MAX) {
    return x + y;
  }
  return x;
}
