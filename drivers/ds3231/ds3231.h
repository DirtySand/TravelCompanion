#ifndef ds3231_h
#define ds3231_h
#include <stdint.h>
#include <stdbool.h>

typedef enum {
  DS3231_OK,
  DS3231_I2C_ERROR,
  DS3231_NULL_PTR,
  DS3231_IS_READY,
  DS3231_TIMEOUT,
  DS3231_TIME_NOT_SET,
  DS3231_ALARM_NOT_SET,
} ds3231_status_t;

typedef struct ds3231 ds3231_t;

typedef enum {
  DS3231_24_HOUR_FORMAT,
  DS3231_12_HOUR_FORMAT,
} ds3231_time_format_t;

typedef int (*ds3231_i2c_read_fn)(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                                  uint8_t *data, uint8_t length);
typedef int (*ds3231_i2c_write_fn)(void *ctx, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data,
                                   uint8_t length);
ds3231_t *ds3231_create(void *ctx, uint8_t dev_addr,
                        ds3231_i2c_read_fn i2c_read,
                        ds3231_i2c_write_fn i2c_write);
ds3231_status_t ds3231_init(ds3231_t *dev);
ds3231_status_t ds3231_check_oscillator_stopped(ds3231_t *dev, bool *stopped);
ds3231_status_t ds3231_set_time_format(ds3231_t *dev, ds3231_time_format_t format);
ds3231_status_t ds3231_read_time(ds3231_t *dev, uint8_t *seconds, uint8_t *minutes, uint8_t *hours);
ds3231_status_t ds3231_set_time(ds3231_t *dev, uint8_t hours, uint8_t minutes, uint8_t seconds);
ds3231_status_t ds3231_read_date(ds3231_t *dev, uint8_t *day, uint8_t *month, uint8_t *year);
ds3231_status_t ds3231_set_date(ds3231_t *dev, uint8_t day, uint8_t month, uint8_t year);
ds3231_status_t ds3231_read_alarm_1(ds3231_t *dev);
ds3231_status_t ds3231_set_alarm_1(ds3231_t *dev, uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t day_of_week);
ds3231_status_t ds3231_read_alarm_2(ds3231_t *dev);
ds3231_status_t ds3231_set_alarm_2(ds3231_t *dev, uint8_t hours, uint8_t minutes, uint8_t day_of_week);

#endif