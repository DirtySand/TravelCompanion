#include "sd_diskio.h"
#include "board_config.h"
#include "ff.h"
#include "diskio.h"
#include <stddef.h>

#define SD_DRIVE 0
#define FAT_YEAR_BASE 1980u

static sdcard_t *sd_card = NULL;
static ds3231_datetime_t fat_time;
static bool fat_time_valid = false;

void sd_diskio_attach(sdcard_t *card) { sd_card = card; }

void sd_diskio_set_time(const ds3231_datetime_t *utc, bool valid) {
  fat_time = *utc;
  fat_time_valid = valid;
}

DWORD get_fattime(void) {
  if (!fat_time_valid) {
    return ((DWORD)(FF_NORTC_YEAR - FAT_YEAR_BASE) << 25) |
           ((DWORD)FF_NORTC_MON << 21) | ((DWORD)FF_NORTC_MDAY << 16);
  }
  const ds3231_datetime_t *t = &fat_time;
  return ((DWORD)(CENTURY_BASE + t->year - FAT_YEAR_BASE) << 25) |
         ((DWORD)t->month << 21) | ((DWORD)t->day << 16) |
         ((DWORD)t->hours << 11) | ((DWORD)t->minutes << 5) |
         ((DWORD)t->seconds >> 1);
}

DSTATUS disk_status(BYTE pdrv) {
  if (pdrv != SD_DRIVE || sd_card == NULL) {
    return STA_NOINIT | STA_NODISK;
  }
  return sdcard_is_ready(sd_card) ? 0 : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE pdrv) {
  if (pdrv != SD_DRIVE || sd_card == NULL) {
    return STA_NOINIT | STA_NODISK;
  }
  sdcard_status_t status = sdcard_init(sd_card);
  if (status == SDCARD_NO_CARD) {
    return STA_NOINIT | STA_NODISK;
  }
  return (status == SDCARD_OK) ? 0 : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
  if (pdrv != SD_DRIVE || sd_card == NULL || count == 0) {
    return RES_PARERR;
  }
  if (!sdcard_is_ready(sd_card)) {
    return RES_NOTRDY;
  }
  for (UINT i = 0; i < count; i++) {
    if (sdcard_read_block(sd_card, (uint32_t)(sector + i),
                          buff + i * SDCARD_BLOCK_SIZE) != SDCARD_OK) {
      return RES_ERROR;
    }
  }
  return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
  if (pdrv != SD_DRIVE || sd_card == NULL || count == 0) {
    return RES_PARERR;
  }
  if (!sdcard_is_ready(sd_card)) {
    return RES_NOTRDY;
  }
  for (UINT i = 0; i < count; i++) {
    if (sdcard_write_block(sd_card, (uint32_t)(sector + i),
                           buff + i * SDCARD_BLOCK_SIZE) != SDCARD_OK) {
      return RES_ERROR;
    }
  }
  return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
  (void)buff;
  if (pdrv != SD_DRIVE || sd_card == NULL) {
    return RES_PARERR;
  }
  if (cmd == CTRL_SYNC) {
    return (sdcard_wait_ready(sd_card) == SDCARD_OK) ? RES_OK : RES_ERROR;
  }
  return RES_PARERR;
}
