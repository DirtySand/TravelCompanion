add_library(bme680 STATIC
    bme680.c
)
target_include_directories(bme680 PUBLIC ${CMAKE_CURRENT_LIST_DIR})
add_library(bme680_port STATIC
    bme680_port.c
)
target_include_directories(bme680_port PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(bme680_port PUBLIC bme680 hardware_i2c)