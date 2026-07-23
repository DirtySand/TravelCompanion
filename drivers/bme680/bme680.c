#include "bme680.h"
#include <stddef.h>

static const uint8_t BME680_REG_CHIP_ID = 0xD0;

struct bme680 {
  void *ctx;
  uint8_t dev_addr;
  bme680_i2c_read_fn i2c_read;
  bme680_i2c_write_fn i2c_write;
};

static bme680_t instance;

static int instance_used = 0;

bme680_t *bme680_create(void *ctx, uint8_t dev_addr,
                        bme680_i2c_read_fn i2c_read,
                        bme680_i2c_write_fn i2c_write) {
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
  instance_used = 1;

  return &instance;
}

bme680_status_t bme680_read_chip_id(bme680_t *dev, uint8_t *chip_id) {
  if (dev == NULL || chip_id == NULL) {
    return BME680_NULL_PTR;
  }

  int result =
      dev->i2c_read(dev->ctx, dev->dev_addr, BME680_REG_CHIP_ID, chip_id, 1);
  if (result < 0) {
    return BME680_I2C_ERROR;
  }

  return BME680_OK;
}