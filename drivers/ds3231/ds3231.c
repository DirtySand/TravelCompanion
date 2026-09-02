#include "ds3231.h"
#include "ds3231_config.h"
#include "ds3231_internal.h"
#include <stddef.h>
#include <stdint.h>

static ds3231_t instance;
static int instance_used = 0;

ds3231_status_t ds3231_init(ds3231_t *dev) {
  if (dev == NULL) {
    return DS3231_NULL_PTR;
  }
  return DS3231_OK;
}

static ds3231_status_t ds3231_write_bit(ds3231_t *dev, uint8_t reg_addr,
                                        uint8_t bit_pos, uint8_t value) {
  uint8_t data;
  if (dev == NULL) {
    return DS3231_NULL_PTR;
  }
  int result = dev->i2c_read(dev->ctx, dev->dev_addr, reg_addr, &data, 1);
  if (result < 0) {
    return DS3231_I2C_ERROR;
  }
  if (value) {
    data |= (1 << bit_pos);
  } else {
    data &= ~(1 << bit_pos);
  }
  result = dev->i2c_write(dev->ctx, dev->dev_addr, reg_addr, &data, 1);
  if (result < 0) {
    return DS3231_I2C_ERROR;
  }
  return DS3231_OK;
}

static ds3231_status_t ds3231_read_bit(ds3231_t *dev, uint8_t reg_addr,
                                       uint8_t bit_pos, bool *value) {
  uint8_t data;
  if (dev == NULL || value == NULL) {
    return DS3231_NULL_PTR;
  }
  int result = dev->i2c_read(dev->ctx, dev->dev_addr, reg_addr, &data, 1);
  if (result < 0) {
    return DS3231_I2C_ERROR;
  }
  *value = (data >> bit_pos) & 0x01;
  return DS3231_OK;
}

ds3231_status_t ds3231_check_oscillator_stopped(ds3231_t *dev, bool *stopped) {
  if (dev == NULL || stopped == NULL) {
    return DS3231_NULL_PTR;
  }
  return ds3231_read_bit(dev, DS3231_STATUS_REG, DS3231_OSF_BIT, stopped);
}

ds3231_t *ds3231_create(void *ctx, uint8_t dev_addr,
                        ds3231_i2c_read_fn i2c_read,
                        ds3231_i2c_write_fn i2c_write) {
  if (i2c_read == NULL || i2c_write == NULL) {
    return NULL;
  }
  if (instance_used) {
    return NULL;
  }
  instance.ctx = ctx;
  instance.dev_addr = dev_addr;
  instance.i2c_read = i2c_read;
  instance.i2c_write = i2c_write;
  if (ds3231_init(&instance) == DS3231_OK) {
    instance_used = 1;
    return &instance;
  }
  return NULL;
}

ds3231_status_t ds3231_set_time_format(ds3231_t *dev,
                                       ds3231_time_format_t format) {
  if (dev == NULL) {
    return DS3231_NULL_PTR;
  }
  return ds3231_write_bit(dev, DS3231_HOUR_REG, DS3231_12_24_BIT,
                          format == DS3231_12_HOUR_FORMAT ? 1 : 0);
}

static void convert_bcd_to_decimal(uint8_t *value) {
  *value = ((*value >> 4) * 10) + (*value & 0x0F);
}

static void convert_decimal_to_bcd(uint8_t *value) {
  *value = ((*value / 10) << 4) | (*value % 10);
}

ds3231_status_t ds3231_set_time(ds3231_t *dev, uint8_t hours, uint8_t minutes,
                                uint8_t seconds) {
  if (dev == NULL) {
    return DS3231_NULL_PTR;
  }
  uint8_t buffer[3] = {seconds, minutes, hours};
  convert_decimal_to_bcd(&buffer[0]);
  convert_decimal_to_bcd(&buffer[1]);
  convert_decimal_to_bcd(&buffer[2]);
  int result =
      dev->i2c_write(dev->ctx, dev->dev_addr, DS3231_SECONDS_REG, buffer, 3);
  ds3231_write_bit(dev, DS3231_STATUS_REG, DS3231_OSF_BIT, 0);
  if (result < 0) {
    return DS3231_I2C_ERROR;
  }
  return DS3231_OK;
}

ds3231_status_t ds3231_set_date(ds3231_t *dev, uint8_t day, uint8_t month,
                                uint8_t year) {
  if (dev == NULL) {
    return DS3231_NULL_PTR;
  }
  uint8_t buffer[3] = {day, month, year};
  convert_decimal_to_bcd(&buffer[0]);
  convert_decimal_to_bcd(&buffer[1]);
  convert_decimal_to_bcd(&buffer[2]);
  int result =
      dev->i2c_write(dev->ctx, dev->dev_addr, DS3231_DATE_REG, buffer, 3);
  if (result < 0) {
    return DS3231_I2C_ERROR;
  }
  return DS3231_OK;
}

ds3231_status_t ds3231_set_alarm_1(ds3231_t *dev, uint8_t hours,
                                   uint8_t minutes, uint8_t seconds,
                                   uint8_t day_or_date,
                                   ds3231_alarm1_rate_t rate) {
  if (dev == NULL) {
    return DS3231_NULL_PTR;
  }
  uint8_t buffer[4] = {seconds, minutes, hours, day_or_date};
  for (int i = 0; i < 4; i++) {
    convert_decimal_to_bcd(&buffer[i]);
  }
  uint8_t alarm_mask;
  bool use_day_of_week = false;
  switch (rate) {
  case DS3231_ALARM1_EVERY_SECOND:
    alarm_mask = 0x0F;
    break;
  case DS3231_ALARM1_MATCH_SECONDS:
    alarm_mask = 0x0E;
    break;
  case DS3231_ALARM1_MATCH_MINUTES_SECONDS:
    alarm_mask = 0x0C;
    break;
  case DS3231_ALARM1_MATCH_HOURS_MINUTES_SECONDS:
    alarm_mask = 0x08;
    break;
  case DS3231_ALARM1_MATCH_DAY_OF_WEEK:
    alarm_mask = 0x10;
    use_day_of_week = true;
    break;
  case DS3231_ALARM1_MATCH_DATE:
  default:
    alarm_mask = 0x00;
  }
  for (int i = 0; i < 4; i++) {
    if (alarm_mask & (1 << i)) {
      buffer[i] |= (1 << DS3231_ALARM_MASK_BIT);
    }
  }
  if (use_day_of_week) {
    buffer[3] |= (1 << DS3231_ALARM_DAY_DATE_BIT);
  }
  int result =
      dev->i2c_write(dev->ctx, dev->dev_addr, DS3231_A1M1_REG, buffer, 4);
  return (result < 0) ? DS3231_I2C_ERROR : DS3231_OK;
}

ds3231_status_t ds3231_read_alarm_1(ds3231_t *dev) {
  if (dev == NULL) {
    return DS3231_NULL_PTR;
  }
  uint8_t buffer[4];
  int result =
      dev->i2c_read(dev->ctx, dev->dev_addr, DS3231_A1M1_REG, buffer, 4);
  if (result < 0) {
    return DS3231_I2C_ERROR;
  }
  dev->alarm1_seconds = buffer[0];
  dev->alarm1_minutes = buffer[1];
  dev->alarm1_hours = buffer[2];
  dev->alarm1_day_of_week = buffer[3];
  convert_bcd_to_decimal(&dev->alarm1_seconds);
  convert_bcd_to_decimal(&dev->alarm1_minutes);
  convert_bcd_to_decimal(&dev->alarm1_hours);
  convert_bcd_to_decimal(&dev->alarm1_day_of_week);
  return DS3231_OK;
}

ds3231_status_t ds3231_set_alarm_2(ds3231_t *dev, uint8_t hours,
                                   uint8_t minutes, uint8_t day_or_date,
                                   ds3231_alarm2_rate_t rate) {
  if (dev == NULL) {
    return DS3231_NULL_PTR;
  }
  uint8_t buffer[3] = {minutes, hours, day_or_date};
  for (int i = 0; i < 3; i++) {
    convert_decimal_to_bcd(&buffer[i]);
  }
  uint8_t alarm_mask;
  bool use_day_of_week = false;
  switch (rate) {
  case DS3231_ALARM2_EVERY_MINUTE:
    alarm_mask = 0x07;
    break;
  case DS3231_ALARM2_MATCH_MINUTES:
    alarm_mask = 0x06;
    break;
  case DS3231_ALARM2_MATCH_HOURS_MINUTES:
    alarm_mask = 0x04;
    break;
  case DS3231_ALARM2_MATCH_DAY_OF_WEEK:
    alarm_mask = 0x00;
    use_day_of_week = true;
    break;
  case DS3231_ALARM2_MATCH_DATE:
  default:
    alarm_mask = 0x00;
  }
  for (int i = 0; i < 3; i++) {
    if (alarm_mask & (1 << i)) {
      buffer[i] |= (1 << DS3231_ALARM_MASK_BIT);
    }
  }
  if (use_day_of_week) {
    buffer[2] |= (1 << DS3231_ALARM_DAY_DATE_BIT);
  }
  int result =
      dev->i2c_write(dev->ctx, dev->dev_addr, DS3231_A2M2_REG, buffer, 3);
  return (result < 0) ? DS3231_I2C_ERROR : DS3231_OK;
}

ds3231_status_t ds3231_read_alarm_2(ds3231_t *dev) {
  if (dev == NULL) {
    return DS3231_NULL_PTR;
  }
  uint8_t buffer[3];
  int result =
      dev->i2c_read(dev->ctx, dev->dev_addr, DS3231_A2M2_REG, buffer, 3);
  if (result < 0) {
    return DS3231_I2C_ERROR;
  }
  dev->alarm2_minutes = buffer[0];
  dev->alarm2_hours = buffer[1];
  dev->alarm2_day_of_week = buffer[2];
  convert_bcd_to_decimal(&dev->alarm2_minutes);
  convert_bcd_to_decimal(&dev->alarm2_hours);
  convert_bcd_to_decimal(&dev->alarm2_day_of_week);
  return DS3231_OK;
}

ds3231_status_t ds3231_read_datetime(ds3231_t *dev,
                                     ds3231_datetime_t *datetime) {
  if (dev == NULL || datetime == NULL) {
    return DS3231_NULL_PTR;
  }
  uint8_t buffer[7];
  int result =
      dev->i2c_read(dev->ctx, dev->dev_addr, DS3231_SECONDS_REG, buffer, 7);
  if (result < 0) {
    return DS3231_I2C_ERROR;
  }
  buffer[0] &= 0x7F;
  buffer[1] &= 0x7F;
  buffer[2] &= 0x3F;
  buffer[3] &= 0x07;
  buffer[4] &= 0x3F;
  buffer[5] &= 0x1F;
  for (int i = 0; i < 7; i++) {
    convert_bcd_to_decimal(&buffer[i]);
  }
  datetime->seconds = buffer[0];
  datetime->minutes = buffer[1];
  datetime->hours = buffer[2];
  datetime->day_of_week = buffer[3];
  datetime->day = buffer[4];
  datetime->month = buffer[5];
  datetime->year = buffer[6];
}