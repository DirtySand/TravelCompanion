add_library(app STATIC
    app_backpack.c
    app_cards.c
    app_common.c
    app_compass.c
    app_gps.c
    app_imu.c
    app_manual.c
    app_nav.c
    app_sampling.c
    app_tools.c
    app_trip.c
    app_waypoints.c
)
target_include_directories(app PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(app PUBLIC
    pico_stdlib hardware_i2c hardware_clocks pico_rand board_config
    bme680_port ssd1351_port ds3231_port sht40_port neo6m_port qmc5883l_port icm20649_port
    buttons power timezone datetime_utils time_sync
    ui compass motion nav wp_flash sample sd_log
)
