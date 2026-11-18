add_library(i2c_bus STATIC i2c_bus.c)
target_include_directories(i2c_bus PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(i2c_bus PUBLIC hardware_i2c hardware_gpio pico_time board_config)
