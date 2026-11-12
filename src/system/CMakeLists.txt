add_library(power STATIC power.c)
target_include_directories(power PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(power PUBLIC hardware_sleep hardware_gpio hardware_clocks pico_stdlib board_config)

add_library(buttons STATIC buttons.c)
target_include_directories(buttons PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(buttons PUBLIC hardware_gpio hardware_sync pico_stdlib board_config)
