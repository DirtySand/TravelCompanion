add_library(ds3231 STATIC
    ds3231.c
)
target_include_directories(ds3231 PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_library(ds3231_port STATIC
    ds3231_port.c
)
target_include_directories(ds3231_port PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(ds3231_port PUBLIC ds3231 hardware_i2c board_config)