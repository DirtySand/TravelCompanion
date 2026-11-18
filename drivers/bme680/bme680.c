#include "bme680.h"
#include <stdbool.h>
#include <stddef.h>
#include "bme680_compensation.h"
#include "bme680_internal.h"

#define BME680_BLOCK_A_START 0x8A
#define BME680_BLOCK_A_END 0xA0
#define BME680_BLOCK_A_LENGTH (BME680_BLOCK_A_END - BME680_BLOCK_A_START + 1)
#define BME680_BLOCK_B_START 0xE1
#define BME680_BLOCK_B_END 0xEA
#define BME680_BLOCK_B_LENGTH (BME680_BLOCK_B_END - BME680_BLOCK_B_START + 1)
#define BME680_IIR_ADDR 0x75 // config_filter<4:2>
#define BME680_CTRL_HUM_ADDR 0x72 // osrs_h<2:0>
#define BME680_CTRL_MEAS_ADDR 0x74 // osrs_t<7:5>, osrs_p<4:2>, mode<1:0>
#define BME680_MEAS_STATUS_ADDR 0x1D // new_data_0<7>, measuring<5>
#define BME680_MEAS_STATUS_NEW_DATA (1u << 7)
#define BME680_MEAS_STATUS_MEASURING (1u << 5)
#define BME680_DATA_START 0x1F // press 0x1F-0x21, temp 0x22-0x24, hum 0x25-0x26
#define BME680_DATA_LENGTH 8
#define BME680_CTRL_MEAS_FORCED 0x25 // osrs_t x1, osrs_p x1, mode forced
#define BME680_FIRST_POLL_MS 5
#define BME680_POLL_MS 2
#define BME680_MEASUREMENT_TIMEOUT_MS 60

static const uint8_t BME680_REG_CHIP_ID = 0xD0;

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

static bme680_status_t bme680_set_calib(bme680_t *dev) {
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

  dev->calib.temp_calib.par_t1 =
      (uint16_t)(buffer_b[0xEA - BME680_BLOCK_B_START] << 8 |
                 buffer_b[0xE9 - BME680_BLOCK_B_START]);
  dev->calib.temp_calib.par_t2 =
      (int16_t)(buffer_a[0x8B - BME680_BLOCK_A_START] << 8 |
                buffer_a[0x8A - BME680_BLOCK_A_START]);
  dev->calib.temp_calib.par_t3 = (int8_t)buffer_a[0x8C - BME680_BLOCK_A_START];

  dev->calib.press_calib.par_p1 =
      (uint16_t)(buffer_a[0x8F - BME680_BLOCK_A_START] << 8 |
                 buffer_a[0x8E - BME680_BLOCK_A_START]);
  dev->calib.press_calib.par_p2 =
      (int16_t)(buffer_a[0x91 - BME680_BLOCK_A_START] << 8 |
                buffer_a[0x90 - BME680_BLOCK_A_START]);
  dev->calib.press_calib.par_p3 = (int8_t)buffer_a[0x92 - BME680_BLOCK_A_START];
  dev->calib.press_calib.par_p4 =
      (int16_t)(buffer_a[0x95 - BME680_BLOCK_A_START] << 8 |
                buffer_a[0x94 - BME680_BLOCK_A_START]);
  dev->calib.press_calib.par_p5 =
      (int16_t)(buffer_a[0x97 - BME680_BLOCK_A_START] << 8 |
                buffer_a[0x96 - BME680_BLOCK_A_START]);
  dev->calib.press_calib.par_p6 = (int8_t)buffer_a[0x99 - BME680_BLOCK_A_START];
  dev->calib.press_calib.par_p7 = (int8_t)buffer_a[0x98 - BME680_BLOCK_A_START];
  dev->calib.press_calib.par_p8 =
      (int16_t)(buffer_a[0x9D - BME680_BLOCK_A_START] << 8 |
                buffer_a[0x9C - BME680_BLOCK_A_START]);
  dev->calib.press_calib.par_p9 =
      (int16_t)(buffer_a[0x9F - BME680_BLOCK_A_START] << 8 |
                buffer_a[0x9E - BME680_BLOCK_A_START]);
  dev->calib.press_calib.par_p10 =
      (uint8_t)buffer_a[0xA0 - BME680_BLOCK_A_START];

  dev->calib.hum_calib.par_h1 =
      (uint16_t)((buffer_b[0xE3 - BME680_BLOCK_B_START]) << 4 |
                 ((buffer_b[0xE2 - BME680_BLOCK_B_START]) & 0x0F));
  dev->calib.hum_calib.par_h2 =
      (uint16_t)((buffer_b[0xE2 - BME680_BLOCK_B_START] >> 4) |
                 (buffer_b[0xE1 - BME680_BLOCK_B_START] << 4));
  dev->calib.hum_calib.par_h3 = (int8_t)buffer_b[0xE4 - BME680_BLOCK_B_START];
  dev->calib.hum_calib.par_h4 = (int8_t)buffer_b[0xE5 - BME680_BLOCK_B_START];
  dev->calib.hum_calib.par_h5 = (int8_t)buffer_b[0xE6 - BME680_BLOCK_B_START];
  dev->calib.hum_calib.par_h6 = (uint8_t)buffer_b[0xE7 - BME680_BLOCK_B_START];
  dev->calib.hum_calib.par_h7 = (int8_t)buffer_b[0xE8 - BME680_BLOCK_B_START];

  return BME680_OK;
}

static bme680_status_t write_register(bme680_t *dev, uint8_t reg,
                                      uint8_t value) {
  int result = dev->i2c_write(dev->ctx, dev->dev_addr, reg, &value, 1);
  return (result < 0) ? BME680_I2C_ERROR : BME680_OK;
}

static bme680_status_t bme680_set_hum_oversampling(bme680_t *dev,
                                                   uint8_t osrs_h) {
  if (dev == NULL) {
    return BME680_NULL_PTR;
  }
  return write_register(dev, BME680_CTRL_HUM_ADDR, osrs_h & 0b00000111);
}

static bme680_status_t bme680_set_config_register(bme680_t *dev,
                                                  uint8_t config) {
  if (dev == NULL) {
    return BME680_NULL_PTR;
  }
  return write_register(dev, BME680_IIR_ADDR, config & 0b00011100);
}

static bme680_status_t bme680_set_ctrl_meas_register(bme680_t *dev,
                                                     uint8_t ctrl_meas) {
  if (dev == NULL) {
    return BME680_NULL_PTR;
  }
  return write_register(dev, BME680_CTRL_MEAS_ADDR, ctrl_meas);
}

static bme680_status_t bme680_set_parameters(bme680_t *dev, uint8_t config,
                                             uint8_t hum_oversampling,
                                             uint8_t ctrl_meas) {
  if (dev == NULL) {
    return BME680_NULL_PTR;
  }
  bme680_status_t status = bme680_set_hum_oversampling(dev, hum_oversampling);
  if (status == BME680_OK) {
    status = bme680_set_config_register(dev, config);
  }
  if (status == BME680_OK) {
    status = bme680_set_ctrl_meas_register(dev, ctrl_meas);
  }
  return status;
}

bme680_status_t bme680_init(bme680_t *dev) {
  if (dev == NULL) {
    return BME680_NULL_PTR;
  }
  return bme680_set_parameters(dev, 0, 1, BME680_CTRL_MEAS_FORCED);
}

static bme680_status_t
bme680_get_measurement_status(bme680_t *dev, bool *measuring, bool *new_data) {
  if (dev == NULL || measuring == NULL || new_data == NULL) {
    return BME680_NULL_PTR;
  }
  uint8_t meas_status;
  int result = dev->i2c_read(dev->ctx, dev->dev_addr, BME680_MEAS_STATUS_ADDR,
                             &meas_status, 1);
  if (result < 0) {
    return BME680_I2C_ERROR;
  }
  *measuring = (meas_status & BME680_MEAS_STATUS_MEASURING) != 0;
  *new_data = (meas_status & BME680_MEAS_STATUS_NEW_DATA) != 0;
  return BME680_OK;
}

static bme680_status_t bme680_start_forced_measurement(bme680_t *dev) {
  if (dev == NULL) {
    return BME680_NULL_PTR;
  }
  return bme680_set_ctrl_meas_register(dev, BME680_CTRL_MEAS_FORCED);
}

static bme680_status_t wait_measurement_done(bme680_t *dev) {
  dev->delay_ms(dev->ctx, BME680_FIRST_POLL_MS);
  uint32_t waited_ms = BME680_FIRST_POLL_MS;
  while (true) {
    bool measuring, new_data;
    bme680_status_t status =
        bme680_get_measurement_status(dev, &measuring, &new_data);
    if (status != BME680_OK) {
      return status;
    }
    if (!measuring) {
      return BME680_OK;
    }
    if (waited_ms >= BME680_MEASUREMENT_TIMEOUT_MS) {
      return BME680_TIMEOUT;
    }
    dev->delay_ms(dev->ctx, BME680_POLL_MS);
    waited_ms += BME680_POLL_MS;
  }
}

bme680_status_t bme680_read_measurement_forced(bme680_t *dev,
                                               float *temperature,
                                               float *pressure,
                                               float *humidity) {
  if (dev == NULL || temperature == NULL || pressure == NULL ||
      humidity == NULL) {
    return BME680_NULL_PTR;
  }
  bme680_status_t status = bme680_start_forced_measurement(dev);
  if (status != BME680_OK) {
    return status;
  }
  status = wait_measurement_done(dev);
  if (status != BME680_OK) {
    return status;
  }

  uint8_t data[BME680_DATA_LENGTH];
  int result = dev->i2c_read(dev->ctx, dev->dev_addr, BME680_DATA_START, data,
                             BME680_DATA_LENGTH);
  if (result < 0) {
    return BME680_I2C_ERROR;
  }
  uint32_t pressure_adc = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) |
                          ((uint32_t)data[2] >> 4);
  uint32_t temperature_adc = ((uint32_t)data[3] << 12) |
                             ((uint32_t)data[4] << 4) |
                             ((uint32_t)data[5] >> 4);
  uint32_t humidity_adc = ((uint32_t)data[6] << 8) | (uint32_t)data[7];

  *temperature = bme680_compensate_temperature(dev, temperature_adc);
  *pressure = bme680_compensate_pressure(dev, pressure_adc);
  *humidity = bme680_compensate_humidity(dev, humidity_adc);
  return BME680_OK;
}

bme680_t *bme680_create(void *ctx, uint8_t dev_addr,
                        bme680_i2c_read_fn i2c_read,
                        bme680_i2c_write_fn i2c_write,
                        bme680_delay_ms_fn delay_ms) {
  if (i2c_read == NULL || i2c_write == NULL || delay_ms == NULL) {
    return NULL;
  }
  if (instance_used) {
    return NULL;
  }
  instance.ctx = ctx;
  instance.dev_addr = dev_addr;
  instance.i2c_read = i2c_read;
  instance.i2c_write = i2c_write;
  instance.delay_ms = delay_ms;
  if (bme680_set_calib(&instance) != BME680_OK ||
      bme680_init(&instance) != BME680_OK) {
    return NULL;
  }
  instance_used = 1;
  return &instance;
}
