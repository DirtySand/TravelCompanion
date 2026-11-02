# FatFs R0.16 by ChaN (http://elm-chan.org/fsw/ff/), files ff.c, ff.h and
# diskio.h copied unmodified; ffconf.h is the project configuration.
# The disk I/O layer (diskio.h functions + get_fattime) is in sd_diskio.c.
add_library(fatfs STATIC ff.c)
target_include_directories(fatfs PUBLIC ${CMAKE_CURRENT_LIST_DIR})
