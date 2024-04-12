#include "led_status.h"

void led_status_init() {
    #if LIB_PICO_CYW43_ARCH
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return -1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    #endif


    gpio_init(PIN_LED_STATUS);
    gpio_set_dir(PIN_LED_STATUS, GPIO_OUT);
    gpio_put(PIN_LED_STATUS, 0);
}

void led_status_set(bool status) {
    #if LIB_PICO_CYW43_ARCH
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, status);
    #else
        gpio_put(PIN_LED_STATUS, status);
    #endif
}

void led_status_blink(uint count, uint32_t on_time_ms, uint32_t off_time_ms) {
    for (int i = 0; i < count; i++) {
        led_status_set(true);
        sleep_ms(on_time_ms);
        led_status_set(false);
        sleep_ms(off_time_ms);
    }
}