add_library(compass STATIC compass.c attitude.c)
target_include_directories(compass PUBLIC ${CMAKE_CURRENT_LIST_DIR})
