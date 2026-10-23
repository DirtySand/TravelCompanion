#ifndef TIMEZONE_H
#define TIMEZONE_H
#include <stdbool.h>
#include <stdint.h>

#define TIMEZONE_STANDARD_OFFSET_SECONDS 3600
#define TIMEZONE_DST_OFFSET_SECONDS 3600

bool is_dst_active(uint8_t day, uint8_t month, uint16_t year, uint8_t hour_utc);
#endif  // TIMEZONE_H