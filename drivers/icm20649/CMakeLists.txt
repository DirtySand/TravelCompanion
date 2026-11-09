add_library(icm20649 STATIC
    icm20649.c
)
target_include_directories(icm20649 PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_library(icm20649_port STATIC
    icm20649_port.c
)
target_include_directories(icm20649_port PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(icm20649_port PUBLIC icm20649 hardware_i2c pico_time)
