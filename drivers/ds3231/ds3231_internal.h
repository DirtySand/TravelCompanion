#ifndef ds3231_internal_h
#define ds3231_internal_h

#include "ds3231.h"

struct ds3231 {
  void *ctx;
  uint8_t dev_addr;
  ds3231_i2c_read_fn i2c_read;
  ds3231_i2c_write_fn i2c_write;
  uint8_t alarm1_seconds, alarm1_minutes, alarm1_hours, alarm1_day_of_week;
  uint8_t alarm2_minutes, alarm2_hours, alarm2_day_of_week;
  bool alarm1_enabled, alarm2_enabled;
};

#endif