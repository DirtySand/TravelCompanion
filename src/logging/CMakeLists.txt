add_library(csv STATIC csv.c)
target_include_directories(csv PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_library(sample STATIC sample.c)
target_include_directories(sample PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(sample PUBLIC csv ds3231 board_config)

add_library(full_record STATIC full_record.c)
target_include_directories(full_record PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(full_record PUBLIC sample csv ds3231)

add_library(log_crypto STATIC log_crypto.c)
target_include_directories(log_crypto PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(log_crypto PUBLIC monocypher)

add_library(sd_log STATIC sd_log.c sd_diskio.c)
target_include_directories(sd_log PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(sd_log PUBLIC sdcard_port fatfs log_crypto sample ds3231 board_config pico_rand pico_stdlib log)
