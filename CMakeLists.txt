cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(travel-companion C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(travel-companion main.c)
target_link_libraries(travel-companion pico_stdlib hardware_i2c)

pico_enable_stdio_usb(travel-companion 1)
pico_enable_stdio_uart(travel-companion 0)
pico_add_extra_outputs(travel-companion)
