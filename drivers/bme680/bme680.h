#ifndef BME680_H
#define BME680_H

#include <stdint.h>

typedef struct bme680 bme680_t;

typedef enum {
  BME680_OK,
  BME680_WRONG_CHIP_ID,
  BME680_I2C_ERROR,
  BME680_NULL_PTR,
} bme680_status_t;

typedef int (*bme680_i2c_read_fn)(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                                  uint8_t *data, uint8_t length);
typedef int (*bme680_i2c_write_fn)(void *ctx, uint8_t dev_addr,
                                   uint8_t reg_addr, const uint8_t *data,
                                   uint8_t length);

bme680_t *bme680_create(void *ctx, uint8_t dev_addr,
                        bme680_i2c_read_fn i2c_read,
                        bme680_i2c_write_fn i2c_write);

bme680_status_t bme680_read_chip_id(bme680_t *dev, uint8_t *chip_id);

#endif