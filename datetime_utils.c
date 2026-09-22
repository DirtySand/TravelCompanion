#include "datetime_utils.h"
#include "drivers/ds3231/ds3231.h"
#include <stdbool.h>
#include <stdio.h>

static bool is_leap_year(uint16_t year) {
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

static uint8_t days_in_month(uint8_t month, uint16_t year) {
  switch (month) {
  case 1:
  case 3:
  case 5:
  case 7:
  case 8:
  case 10:
  case 12:
    return 31;
  case 4:
  case 6:
  case 9:
  case 11:
    return 30;
  case 2:
    return is_leap_year(year) ? 29 : 28;
  default:
    return 0;
  }
}

void datetime_add_seconds(ds3231_datetime_t *datetime, int seconds,
                          uint16_t century_base) {
  if (datetime == NULL) {
    return;
  }
  uint32_t total_seconds = datetime->seconds + seconds;
  datetime->seconds = total_seconds % 60;
  uint32_t total_minutes = datetime->minutes + (total_seconds / 60);
  datetime->minutes = total_minutes % 60;
  uint32_t total_hours = datetime->hours + (total_minutes / 60);
  datetime->hours = total_hours % 24;
  uint32_t total_days = (total_hours / 24);
  uint16_t year = century_base + datetime->year;
  while (total_days > 0) {
    uint8_t dim = days_in_month(datetime->month, year);
    if (datetime->day < dim) {
      datetime->day++;
    } else {
      datetime->day = 1;
      if (datetime->month < 12) {
        datetime->month++;
      } else {
        datetime->month = 1;
        year++;
        datetime->year = year - century_base;
      }
    }
    total_days--;
  }
}