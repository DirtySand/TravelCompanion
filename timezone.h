#ifndef TIMEZONE_H
#define TIMEZONE_H
#include <stdbool.h>
#include <stdint.h>

bool is_dst_active(uint8_t day, uint8_t month, uint16_t year, uint8_t hour_utc);
#endif  // TIMEZONE_H