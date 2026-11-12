add_library(ui STATIC ui.c cards.c)
target_include_directories(ui PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(ui PUBLIC ssd1351 ds3231 neo6m datetime_utils board_config)
