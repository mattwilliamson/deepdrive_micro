cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(deepdrive_micro C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

include_directories(
    include
    src/imu
)


link_directories(libmicroros)
add_executable(deepdrive_micro
    src/main.cpp
    src/pico_uart_transport.c
    src/led_ring.cpp
    src/led_status.c
    src/servo.c
    src/status.cpp
    src/motor.cpp
    src/imu.cpp
    src/pid.cpp
    src/analog_sensors.cpp

    # IMU
    src/imu/pico-icm20948.c
    src/imu/MadgwickAHRS.c
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
    libmicroros/include
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

# target_link_libraries(deepdrive_micro PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(deepdrive_micro)

