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
  return ds3231_read_bit(dev, STATUS_REG, DS3231_OSF_BIT, stopped);
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

void convert_bcd_to_decimal(uint8_t *value) {
  *value = ((*value >> 4) * 10) + (*value & 0x0F);
}

void convert_decimal_to_bcd(uint8_t *value) {
  *value = ((*value / 10) << 4) | (*value % 10);
}

ds3231_status_t ds3231_read_time(ds3231_t *dev, uint8_t *seconds, uint8_t *minutes, uint8_t *hours) {
  if (dev == NULL || seconds == NULL || minutes == NULL || hours == NULL) {
    return DS3231_NULL_PTR;
  }
  uint8_t buffer[3];
  int result =
      dev->i2c_read(dev->ctx, dev->dev_addr, DS3231_SECONDS_REG, buffer, 3);
  if (result < 0) {
    return DS3231_I2C_ERROR;
  }
  convert_bcd_to_decimal(&buffer[0]);
  convert_bcd_to_decimal(&buffer[1]);
  convert_bcd_to_decimal(&buffer[2]);
  *seconds = buffer[0];
  *minutes = buffer[1];
  *hours = buffer[2];

  return DS3231_OK;
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
  if (result < 0) {
    return DS3231_I2C_ERROR;
  }
  return DS3231_OK;
}

ds3231_status_t ds3231_read_date(ds3231_t *dev, uint8_t *day, uint8_t *month, uint8_t *year) {
  if (dev == NULL || day == NULL || month == NULL || year == NULL) {
    return DS3231_NULL_PTR;
  }
  uint8_t buffer[3];
  int result =
      dev->i2c_read(dev->ctx, dev->dev_addr, DS3231_DATE_REG, buffer, 3);
  if (result < 0) {
    return DS3231_I2C_ERROR;
  }
  convert_bcd_to_decimal(&buffer[0]);
  convert_bcd_to_decimal(&buffer[1]);
  convert_bcd_to_decimal(&buffer[2]);
  *day = buffer[0];
  *month = buffer[1];
  *year = buffer[2];

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
  dev->alarm_1.seconds = buffer[0];
  dev->alarm_1.minutes = buffer[1];
  dev->alarm_1.hours = buffer[2];
  dev->alarm_1.day_of_week = buffer[3];
  convert_bcd_to_decimal(&dev->alarm_1.seconds);
  convert_bcd_to_decimal(&dev->alarm_1.minutes);
  convert_bcd_to_decimal(&dev->alarm_1.hours);
  convert_bcd_to_decimal(&dev->alarm_1.day_of_week);
  return DS3231_OK;
}

ds3231_status_t ds3231_set_alarm_1(ds3231_t *dev, uint8_t hours, //TODO: set bit 6
                                   uint8_t minutes, uint8_t seconds,
                                   uint8_t day_of_week) {
  if (dev == NULL) {
    return DS3231_NULL_PTR;
  }
  convert_decimal_to_bcd(&seconds);
  convert_decimal_to_bcd(&minutes);
  convert_decimal_to_bcd(&hours);
  convert_decimal_to_bcd(&day_of_week);
  int result_seconds =
      dev->i2c_write(dev->ctx, dev->dev_addr, DS3231_A1M1_REG, &seconds, 1);
  int result_minutes =
      dev->i2c_write(dev->ctx, dev->dev_addr, DS3231_A1M2_REG, &minutes, 1);
  int result_hours =
      dev->i2c_write(dev->ctx, dev->dev_addr, DS3231_A1M3_REG, &hours, 1);
  int result_dow =
      dev->i2c_write(dev->ctx, dev->dev_addr, DS3231_A1M4_REG, &day_of_week, 1);

  if (result_seconds < 0 || result_minutes < 0 || result_hours < 0 ||
      result_dow < 0) {
    return DS3231_I2C_ERROR;
  }
  return DS3231_OK;
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
  dev->alarm_2.minutes = buffer[0];
  dev->alarm_2.hours = buffer[1];
  dev->alarm_2.day_of_week = buffer[2];
  convert_bcd_to_decimal(&dev->alarm_2.minutes);
  convert_bcd_to_decimal(&dev->alarm_2.hours);
  convert_bcd_to_decimal(&dev->alarm_2.day_of_week);
  return DS3231_OK;
}

ds3231_status_t ds3231_set_alarm_2(ds3231_t *dev, uint8_t minutes, //TODO: set bit 6
                                   uint8_t hours, uint8_t date) {
  if (dev == NULL) {
    return DS3231_NULL_PTR;
  }
  convert_decimal_to_bcd(&minutes);
  convert_decimal_to_bcd(&hours);
  convert_decimal_to_bcd(&date);
  int result_minuites =
      dev->i2c_write(dev->ctx, dev->dev_addr, DS3231_A2M2_REG, &minutes, 1);
  int result_hours =
      dev->i2c_write(dev->ctx, dev->dev_addr, DS3231_A2M3_REG, &hours, 1);
  int result_date =
      dev->i2c_write(dev->ctx, dev->dev_addr, DS3231_A2M4_REG, &date, 1);
  if (result_minuites < 0 || result_hours < 0 || result_date < 0) {
    return DS3231_I2C_ERROR;
  }
  return DS3231_OK;
}