#include "led_ring.h"

static inline void led_ring_put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

// Return gamma-corrected value for RGB combined
static inline uint32_t urgb_u32_g(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (gamma_u8(r)) << 8) |
            ((uint32_t) (gamma_u8(g)) << 16) |
            (uint32_t) (gamma_u8(b));
}

void led_ring_pattern_snakes(uint len, uint t) {
    for (uint i = 0; i < len; ++i) {
        uint x = (i + (t >> 1)) % 64;
        if (x < 10)
            led_ring_put_pixel(urgb_u32(0xff, 0, 0));
        else if (x >= 15 && x < 25)
            led_ring_put_pixel(urgb_u32(0, 0xff, 0));
        else if (x >= 30 && x < 40)
            led_ring_put_pixel(urgb_u32(0, 0, 0xff));
        else
            led_ring_put_pixel(0);
    }
}

void led_ring_pattern_random(uint len, uint t) {
    if (t % 8)
        return;
    for (int i = 0; i < len; ++i)
        led_ring_put_pixel(rand());
}

void led_ring_pattern_sparkle(uint len, uint t) {
    if (t % 8)
        return;
    for (int i = 0; i < len; ++i)
        led_ring_put_pixel(rand() % 16 ? 0 : 0xffffffff);
}

void led_ring_pattern_greys(uint len, uint t) {
    int max = 100; // let's not draw too much current!
    t %= max;
    for (int i = 0; i < len; ++i) {
        led_ring_put_pixel(t * 0x10101);
        if (++t >= max) t = 0;
    }
}

void led_ring_fill(uint8_t r, uint8_t g, uint8_t b) {
    for (uint i = 0; i < LED_RING_NUM_PIXELS; ++i) {
        led_ring_put_pixel(urgb_u32(gamma_u8(r), gamma_u8(g), gamma_u8(b)));
    }
}

void led_ring_spin(uint8_t r, uint8_t g, uint8_t b, uint t) {
    for (uint i = 0; i < LED_RING_NUM_PIXELS; ++i) {
        if (i == t % LED_RING_NUM_PIXELS) {
            led_ring_put_pixel(urgb_u32(gamma_u8(r), gamma_u8(g), gamma_u8(b)));
        } else {
            led_ring_put_pixel(0);
        }
    }
}

static inline uint8_t gamma_u8(uint8_t x) {
    return led_ring_gamma_table[x];
}

static inline uint8_t sine(uint8_t x) {
    return led_ring_sine_table[x];
}

static inline uint8_t add_u8_max(uint8_t x, uint8_t y) {
    if (x + y < UINT8_MAX) {
        return x + y;
    }
    return x;
}


/**
 * Fades the LED ring from black to a given color. Each color increases in intensity at the same rate.
 *
 * @param r The target red color value.
 * @param g The target green color value.
 * @param b The target blue color value.
 * @param t The time parameter.
 */
void led_ring_fade(uint8_t r, uint8_t g, uint8_t b, uint8_t t) {
    uint8_t r2 = sine((uint8_t)((uint16_t)t * r / UINT8_MAX));
    uint8_t g2 = sine((uint8_t)((uint16_t)t * g / UINT8_MAX));
    uint8_t b2 = sine((uint8_t)((uint16_t)t * b / UINT8_MAX));
    
    led_ring_fill(r2, g2, b2);
}

void led_ring_fade_r(uint8_t r, uint8_t t) {
    uint8_t r2 = sine((uint8_t)((uint16_t)t * r / UINT8_MAX));
    
    led_ring_fill(r2, 0, 0);
}

void led_ring_fade_w(uint8_t w, uint8_t t) {
    uint8_t r2 = sine((uint8_t)((uint16_t)t * w / UINT8_MAX));
    
    led_ring_fill(r2, r2, r2);
}

typedef void (*pattern)(uint len, uint t);
const struct {
    pattern pat;
    const char *name;
} pattern_table[] = {
        {led_ring_pattern_snakes,  "Snakes!"},
        {led_ring_pattern_random,  "Random data"},
        {led_ring_pattern_sparkle, "Sparkles"},
        {led_ring_pattern_greys,   "Greys"},
};

void led_ring_init() {
    stdio_init_all();
    printf("WS2812 Smoke Test, using pin %d", WS2812_PIN);

    PIO pio = pio0;
    int sm = pio_claim_unused_sm(pio, false);
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, LED_RING_IS_RGBW);
}

void led_ring_render(state_t state) {
    static uint8_t t = 0;
    // led_ring_pattern_snakes(LED_RING_NUM_PIXELS, t++);
    // led_ring_pattern_greys(LED_RING_NUM_PIXELS, t++);
    // led_ring_pattern_random(LED_RING_NUM_PIXELS, t++);
    // led_ring_pattern_sparkle(LED_RING_NUM_PIXELS, t++);
    // led_ring_pattern_greys(LED_RING_NUM_PIXELS, t++);
    // led_ring_fill(0x33, 0x00, 0x66);
    // led_ring_fade(0x99, 0x00, 0x00, t++);

    
    // led_ring_spin(0x99, 0x00, 0x00, t++);
    // led_ring_spin(0x00, 0xAA, 0x00, t++);
    // led_ring_spin(0x00, 0x00, 0xAA, t++);

    switch (state) {
        // case STATE_INIT:
        //     led_ring_fade(0x99, 0x99, 0x99, t++);
        //     break;
        case STATE_CONNECTING:
            // led_ring_spin(0xCC, 0xCC, 0xCC, t);
            // led_ring_fade(0x66, 0x66, 0x66, t++);
            led_ring_fade_w(0x66, t);
            break;
        case STATE_CONNECTED:
            led_ring_fill(0x00, 0x99, 0x00);
            break;
        case STATE_ERROR:
            led_ring_fade_r(0x99, t);
            break;
        case STATE_ACTIVE:
            led_ring_fill(0x66, 0x66, 0x66);
            break;
        default:
            // Handle other states or provide a default behavior
            led_ring_fill(0x33, 0x33, 0x33);
            break;
    }

    t++;
}

// int led_loop() {
//     //set_sys_clock_48();
//     stdio_init_all();
//     printf("WS2812 Smoke Test, using pin %d", WS2812_PIN);

//     // todo get free sm
//     PIO pio = pio0;
//     int sm = LED_RING_STATE_MACHINE;
//     uint offset = pio_add_program(pio, &ws2812_program);

//     ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, LED_RING_IS_RGBW);

//     int t = 0;
//     while (1) {
//         int pat = rand() % count_of(pattern_table);
//         int dir = (rand() >> 30) & 1 ? 1 : -1;
//         puts(pattern_table[pat].name);
//         puts(dir == 1 ? "(forward)" : "(backward)");
//         for (int i = 0; i < 1000; ++i) {
//             pattern_table[pat].pat(LED_RING_NUM_PIXELS, t);
//             sleep_ms(10);
//             t += dir;
//         }
//     }
// }