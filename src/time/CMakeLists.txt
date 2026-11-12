add_library(timezone STATIC timezone.c)
target_include_directories(timezone PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_library(datetime_utils STATIC datetime_utils.c)
target_include_directories(datetime_utils PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(datetime_utils PUBLIC ds3231 PRIVATE timezone)

add_library(time_sync STATIC time_sync.c)
target_include_directories(time_sync PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(time_sync PUBLIC ds3231 neo6m)
