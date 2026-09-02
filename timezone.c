
#include <stdbool.h>
#include <stdint.h>

#define ZELLER_SUNDAY 1

static uint8_t day_of_week_for_date(uint8_t day, uint8_t month, uint16_t year) {
  if (month == 1 || month == 2) {
    year -= 1;
    month += 12;
  }
  int day_of_week =
      (day + 13 * (month + 1) / 5 + year % 100 + (year % 100) / 4 +
       (year / 100) / 4 - 2 * (year / 100)) %
      7;
  if (day_of_week < 0) {
    day_of_week += 7;
  }
  return (uint8_t)day_of_week;
}

static uint8_t last_sunday_of_month(uint8_t month, uint16_t year) {
  uint8_t last_day = 31;
  uint8_t weekday = day_of_week_for_date(last_day, month, year);
  uint8_t days_back = (weekday + 7 - ZELLER_SUNDAY) % 7;
  return last_day - days_back;
}

bool is_dst_active(uint8_t day, uint8_t month, uint16_t year,
                   uint8_t hour_utc) {
  if (month < 3 || month > 10) {
    return false;
  }
  if (month > 3 && month < 10) {
    return true;
  }
  if (month == 3) {
    uint8_t dst_start_day = last_sunday_of_month(3, year);
    if (day < dst_start_day) {
      return false;
    }
    if (day > dst_start_day) {
      return true;
    }
    return hour_utc >= 1;
  }
  uint8_t dst_end_day = last_sunday_of_month(10, year);
  if (day < dst_end_day) {
    return true;
  }
  if (day > dst_end_day) {
    return false;
  }
  return hour_utc < 1;
}
