#ifndef ds3231_internal_h
#define ds3231_internal_h

#include "ds3231.h"

struct ds3231_time {
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
};

struct ds3231_date {
  uint8_t day;
  uint8_t month;
  uint8_t year;
};

struct ds3231_alarm_1 {
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint8_t day_of_week;
};

struct ds3231_alarm_2 {
  uint8_t hours;
  uint8_t minutes;
  uint8_t day_of_week;
};

struct ds3231 {
  void *ctx;
  uint8_t dev_addr;
  ds3231_i2c_read_fn i2c_read;
  ds3231_i2c_write_fn i2c_write;
  struct ds3231_time time;
  struct ds3231_date date;
  struct ds3231_alarm_1 alarm_1;
  struct ds3231_alarm_2 alarm_2;
};

#endif