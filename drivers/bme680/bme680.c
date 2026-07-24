#include "bme680.h"
#include <stddef.h>

#define BME680_BLOCK_A_START 0x8A //par_t2 8A, 8B // par_t3 8C // par_p1 8E, 8F // par_p2 90, 91 // par_p3 92 // par_p4 94, 95 // par_p5 96, 97 // par_p7 98 // par_p6 99 // par_p8 9C, 9D // par_p9 9E, 9F // par_p10 A0
#define BME680_BLOCK_A_END 0xA0
#define BME680_BLOCK_A_LENGTH (BME680_BLOCK_A_END - BME680_BLOCK_A_START + 1)

#define BME680_BLOCK_B_START 0xE1 // par_h2 E2<7:4>, E1 // par_h1 E2<3:0>, E3 // par_h3 E4 // par_h4 E5 // par_h5 E6 // par_h6 E7 // par_h7 E8 //par_t1 E9, EA 
#define BME680_BLOCK_B_END 0xEA
#define BME680_BLOCK_B_LENGTH (BME680_BLOCK_B_END - BME680_BLOCK_B_START + 1)

static const uint8_t BME680_REG_CHIP_ID = 0xD0;

struct bme680_calib_temperature {
  uint16_t par_t1;
  int16_t par_t2;
  int8_t par_t3;
};

struct bme680_calib_pressure {
  uint16_t par_p1;
  int16_t par_p2;
  int8_t par_p3;
  int16_t par_p4;
  int16_t par_p5;
  uint8_t par_p6;
  int8_t par_p7;
  int16_t par_p8;
  int16_t par_p9;
  uint8_t par_p10;
};

struct bme680_calib_humidity {
  uint16_t par_h1;
  uint16_t par_h2;
  int8_t par_h3;
  int8_t par_h4;
  int8_t par_h5;
  uint8_t par_h6;
  int8_t par_h7;
};

struct bme680_calib {
  struct bme680_calib_temperature temp_calib;
  struct bme680_calib_pressure press_calib;
  struct bme680_calib_humidity hum_calib;
};

struct bme680 {
  void *ctx;
  uint8_t dev_addr;
  struct bme680_calib calib;
  bme680_i2c_read_fn i2c_read;
  bme680_i2c_write_fn i2c_write;
};

static bme680_t instance;

static int instance_used = 0;


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

bme680_status_t bme680_set_calib(bme680_t *dev) {
  if (dev == NULL) {
    return BME680_NULL_PTR;
  }

  uint8_t buffer_a[BME680_BLOCK_A_LENGTH];
  uint8_t buffer_b[BME680_BLOCK_B_LENGTH];

  int result = dev->i2c_read(dev->ctx, dev->dev_addr, BME680_BLOCK_A_START,
                             buffer_a, BME680_BLOCK_A_LENGTH);
  if (result < 0) {
    return BME680_I2C_ERROR;
  }

  result = dev->i2c_read(dev->ctx, dev->dev_addr, BME680_BLOCK_B_START,
                         buffer_b, BME680_BLOCK_B_LENGTH);
  if (result < 0) {
    return BME680_I2C_ERROR;
  }

  dev->calib.temp_calib.par_t1 = (uint16_t)(buffer_b[0xEA- BME680_BLOCK_B_START] << 8 | buffer_b[0xE9 - BME680_BLOCK_B_START]);
  dev->calib.temp_calib.par_t2 = (int16_t)(buffer_a[0x8B - BME680_BLOCK_A_START] << 8 | buffer_a[0x8A - BME680_BLOCK_A_START]);
  dev->calib.temp_calib.par_t3 = (int8_t)buffer_a[0x8C - BME680_BLOCK_A_START];

  dev->calib.press_calib.par_p1 = (uint16_t)(buffer_a[0x8F - BME680_BLOCK_A_START] << 8 | buffer_a[0x8E - BME680_BLOCK_A_START]);
  dev->calib.press_calib.par_p2 = (int16_t)(buffer_a[0x91 - BME680_BLOCK_A_START] << 8 | buffer_a[0x90 - BME680_BLOCK_A_START]);
  dev->calib.press_calib.par_p3 = (int8_t)buffer_a[0x92 - BME680_BLOCK_A_START];
  dev->calib.press_calib.par_p4 = (int16_t)(buffer_a[0x95 - BME680_BLOCK_A_START] << 8 | buffer_a[0x94 - BME680_BLOCK_A_START]);
  dev->calib.press_calib.par_p5 = (int16_t)(buffer_a[0x97 - BME680_BLOCK_A_START] << 8 | buffer_a[0x96 - BME680_BLOCK_A_START]);
  dev->calib.press_calib.par_p6 = (uint8_t)buffer_a[0x99 - BME680_BLOCK_A_START];
  dev->calib.press_calib.par_p7 = (int8_t)buffer_a[0x98 - BME680_BLOCK_A_START];
  dev->calib.press_calib.par_p8 = (int16_t)(buffer_a[0x9D - BME680_BLOCK_A_START] << 8 | buffer_a[0x9C - BME680_BLOCK_A_START]);
  dev->calib.press_calib.par_p9 = (int16_t)(buffer_a[0x9F - BME680_BLOCK_A_START] << 8 | buffer_a[0x9E - BME680_BLOCK_A_START]);
  dev->calib.press_calib.par_p10 = (uint8_t)buffer_a[0xA0 - BME680_BLOCK_A_START];

  dev->calib.hum_calib.par_h1 = (uint16_t)((buffer_b[0xE3 - BME680_BLOCK_B_START]) << 4 | ((buffer_b[0xE2 - BME680_BLOCK_B_START]) & 0x0F));
  dev->calib.hum_calib.par_h2 = (uint16_t)((buffer_b[0xE2 - BME680_BLOCK_B_START] >> 4) | (buffer_b[0xE1 - BME680_BLOCK_B_START] << 4));
  dev->calib.hum_calib.par_h3 = (int8_t)buffer_b[0xE4 - BME680_BLOCK_B_START];
  dev->calib.hum_calib.par_h4 = (int8_t)buffer_b[0xE5 - BME680_BLOCK_B_START];
  dev->calib.hum_calib.par_h5 = (int8_t)buffer_b[0xE6 - BME680_BLOCK_B_START];
  dev->calib.hum_calib.par_h6 = (uint8_t)buffer_b[0xE7 - BME680_BLOCK_B_START];
  dev->calib.hum_calib.par_h7 = (int8_t)buffer_b[0xE8 - BME680_BLOCK_B_START];

  return BME680_OK;
}

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
  if (bme680_set_calib(&instance) != BME680_OK) {
    return NULL;
  }
  instance_used = 1;
  return &instance;
}