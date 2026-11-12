add_library(motion STATIC steps.c trip.c pdr.c)
target_include_directories(motion PUBLIC ${CMAKE_CURRENT_LIST_DIR})
