#ifndef SD_LOG_H
#define SD_LOG_H

#include "drivers/ds3231/ds3231.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
  SD_LOG_NO_CARD,
  SD_LOG_READY,
  SD_LOG_ERROR,
} sd_log_state_t;

// The OLED SPI port must be initialised first (shared SPI bus).
void sd_log_init(void);

// now_s: RTC time in seconds (datetime_to_seconds), used for the flush and
// retry intervals because the system timer stops while dormant.
void sd_log_append(const char *line, size_t length,
                   const ds3231_datetime_t *utc, bool time_valid,
                   uint32_t now_s);
bool sd_log_flush(uint32_t now_s);
sd_log_state_t sd_log_state(void);

#endif // SD_LOG_H
