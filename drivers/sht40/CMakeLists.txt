add_library(sht40 STATIC
    sht40.c
)
target_include_directories(sht40 PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_library(sht40_port STATIC
    sht40_port.c
)
target_include_directories(sht40_port PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(sht40_port PUBLIC sht40 hardware_i2c pico_time)