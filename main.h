#pragma once

/*
 * C++ HEADERS
 */
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstdint>
#include <cstring>

extern "C" {
    // Error codes can be found in
    // micro_ros_raspberrypi_pico_sdk/libmicroros/include/rcl/types.h
    // and micro_ros_raspberrypi_pico_sdk/libmicroros/include/rmw/ret_types.h
    #include <rcl/error_handling.h>
    #include <rcl/rcl.h>
    #include <rclc/executor.h>
    #include <rclc/rclc.h>
    #include <rmw_microros/rmw_microros.h>
    #include <std_msgs/msg/int32.h>


    // /*
    //  * PICO HEADERS
    //  */
    // #include "pico/stdlib.h"
    // #include "pico/binary_info.h"
    // #include "hardware/gpio.h"
    // #include "hardware/i2c.h"
    // #include "hardware/spi.h"
    // #include "hardware/adc.h"
    // #include "hardware/uart.h"
    #include "hardware/pwm.h"
    #include "pico/stdlib.h"
    #include "pico_uart_transports.h"
    #include "pico/multicore.h"
    #include "pico/time.h"
    #include "hardware/watchdog.h"

    #include "led_status.h"
}

#include "config.h"
#include "status.h"
#include "led_ring.h"