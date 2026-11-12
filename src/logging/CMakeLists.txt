add_library(sample STATIC sample.c)
target_include_directories(sample PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(sample PUBLIC ds3231 board_config)

add_library(log_crypto STATIC log_crypto.c)
target_include_directories(log_crypto PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(log_crypto PUBLIC monocypher)

add_library(sd_log STATIC sd_log.c sd_diskio.c)
target_include_directories(sd_log PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(sd_log PUBLIC sdcard_port fatfs log_crypto sample ds3231 board_config pico_rand pico_stdlib)
