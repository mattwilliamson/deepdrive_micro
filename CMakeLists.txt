cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(deepdrive_micro C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# RP2040 repeating timer heap min=0, max=255, default=16
add_compile_definitions(PICO_TIME_DEFAULT_ALARM_POOL_MAX_TIMERS=32)

pico_sdk_init()

file(GLOB_RECURSE fusion_files "lib/Fusion/*.c")

include_directories(
    include
    generated
    src/imu
    lib/SparkFun_ICM-20948_ArduinoLibrary/src
    lib/pico-icm20948
    lib/Fusion
    lib/hc-sr04
    lib
)

link_directories(lib/libmicroros)

add_executable(deepdrive_micro
    src/main.cpp
    # src/led_ring_main.c

    src/pico_uart_transport.c
    src/memory_usage.c

    src/node.cpp
    src/control_loop.cpp
    src/main_loop.cpp

    # src/param_server.cpp

    src/status.cpp

    src/led_ring.cpp
    src/led_status.c
    src/motor.cpp
    src/motor_manager.cpp
    src/imu.cpp
    src/pid.cpp
    src/analog_sensors.cpp
    src/watchdog.cpp
    lib/hc-sr04/HCSR04.cpp
    
    src/pubsub/pubsub.cpp

    # Subscribers
    src/pubsub/sub_cmd_vel.cpp
    src/pubsub/sub_wheel_speed.cpp

    # Publishers
    src/pubsub/pub_odom.cpp
    src/pubsub/pub_telemetry.cpp
    src/pubsub/pub_battery_state.cpp
    src/pubsub/pub_imu.cpp
    src/pubsub/pub_joint_state.cpp
    src/pubsub/pub_wheel_speed.cpp
    src/pubsub/pub_sonar.cpp

    # IMU
    # lib/SparkFun_ICM-20948_ArduinoLibrary/src/util/ICM_20948_C.c
    # lib/SparkFun_ICM-20948_ArduinoLibrary/src/ICM_20948.cpp
    lib/pico-icm20948/pico-icm20948.c

    # IMU Fusion
    ${fusion_files}

    # Buzzer
    src/buzzer.cpp
    lib/pwm-tone/pwm-tone.c
)

# To add to this, here are examples: `cat /usr/src/pico-sdk/test/kitchen_sink/CMakeLists.txt | grep pwm`
# /usr/src/pico-sdk/test/kitchen_sink/kitchen_sink.c
# #if LIB_PICO_CYW43_ARCH
# grep -i cyw43 /usr/src/pico-sdk/test/kitchen_sink/CMakeLists.txt 
target_link_libraries(deepdrive_micro
    pico_stdlib
    microros
    hardware_pio
    hardware_pwm
    hardware_clocks
    pico_multicore
    hardware_adc
    hardware_i2c
)

if (TARGET pico_cyw43_arch)
target_link_libraries(deepdrive_micro
    pico_cyw43_arch_none
)
endif()

target_include_directories(deepdrive_micro PUBLIC
    lib/libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(deepdrive_micro 1)
pico_enable_stdio_uart(deepdrive_micro 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(deepdrive_micro)

##############
# LED Ring
file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(deepdrive_micro ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

##############
# HC-SR04 Ultrasonic rangefinder
pico_generate_pio_header(deepdrive_micro ${CMAKE_CURRENT_LIST_DIR}/lib/hc-sr04/HCSR04.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)


# target_link_libraries(deepdrive_micro PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(deepdrive_micro)

