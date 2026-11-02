# Monocypher 4.0.3 by Loup Vaillant (https://monocypher.org), files
# monocypher.c and monocypher.h copied unmodified from src/.
add_library(monocypher STATIC monocypher.c)
target_include_directories(monocypher PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_compile_options(monocypher PRIVATE -O2)
