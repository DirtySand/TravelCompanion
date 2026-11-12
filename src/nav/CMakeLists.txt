add_library(nav STATIC geo.c fusion.c alt.c measure.c sun.c waypoints.c)
target_include_directories(nav PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_library(wp_flash STATIC wp_flash.c)
target_include_directories(wp_flash PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(wp_flash PUBLIC nav hardware_flash pico_flash pico_stdlib)
