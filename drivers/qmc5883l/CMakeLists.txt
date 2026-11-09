add_library(qmc5883l STATIC
    qmc5883l.c
)
target_include_directories(qmc5883l PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_library(qmc5883l_port STATIC
    qmc5883l_port.c
)
target_include_directories(qmc5883l_port PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(qmc5883l_port PUBLIC qmc5883l hardware_i2c pico_time)
