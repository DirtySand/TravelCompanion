add_library(neo6m STATIC neo6m.c)
target_include_directories(neo6m PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_library(neo6m_port STATIC neo6m_port.c)
target_include_directories(neo6m_port PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(neo6m_port PUBLIC neo6m hardware_uart hardware_irq hardware_gpio board_config)
