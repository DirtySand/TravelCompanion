add_library(sdcard STATIC sdcard.c)
target_include_directories(sdcard PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_library(sdcard_port STATIC sdcard_port.c)
target_include_directories(sdcard_port PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(sdcard_port PUBLIC sdcard board_config hardware_spi hardware_gpio pico_stdlib)
