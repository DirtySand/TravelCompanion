#ifndef SD_DISKIO_H
#define SD_DISKIO_H

#include "ds3231.h"
#include "sdcard.h"
#include <stdbool.h>

// FatFs disk I/O layer (drive 0 = SD card) and get_fattime().
void sd_diskio_attach(sdcard_t *card);
void sd_diskio_set_time(const ds3231_datetime_t *utc, bool valid);

#endif // SD_DISKIO_H
