#include "bme680.h"
#include <stdbool.h>
#include <stddef.h>
#include "bme680_compensation.h"
#include "bme680_internal.h"

#define BME680_BLOCK_A_START  0x8A // par_t2 8A, 8B // par_t3 8C // par_p1 8E, 8F // par_p2 90, 91 // par_p3 92 // par_p4 94, 95 // par_p5 96, 97 // par_p7 98 // par_p6 99 // par_p8 9C, 9D // par_p9 9E, 9F // par_p10 A0
#define BME680_BLOCK_A_END 0xA0
#define BME680_BLOCK_A_LENGTH (BME680_BLOCK_A_END - BME680_BLOCK_A_START + 1)
#define BME680_BLOCK_B_START 0xE1 // par_h2 E2<7:4>, E1 // par_h1 E2<3:0>, E3 // par_h3 E4 // par_h4 E5 // par_h5 E6 // par_h6 E7 // par_h7 E8 //par_t1 E9, EA
#define BME680_BLOCK_B_END 0xEA
#define BME680_BLOCK_B_LENGTH (BME680_BLOCK_B_END - BME680_BLOCK_B_START + 1)
#define BME680_IIR_ADDR 0x75 // config_filter<4:2>
#define BME680_CTRL_HUM_ADDR 0x72 // osrs_h<2:0>
#define BME680_CTRL_MEAS_ADDR 0x74 // osrs_t<7:5>, osrs_p<4:2>, mode<1:0>
#define BME680_PRESSURE_MSB_ADDR 0x1F
#define BME680_PRESSURE_LSB_ADDR 0x20
#define BME680_PRESSURE_XLSB_ADDR 0x21 // 7:4
#define BME680_TEMPERATURE_MSB_ADDR 0x22
#define BME680_TEMPERATURE_LSB_ADDR 0x23
#define BME680_TEMPERATURE_XLSB_ADDR 0x24 // 7:4
#define BME680_HUMIDITY_MSB_ADDR 0x25
#define BME680_HUMIDITY_LSB_ADDR 0x26
#define BME680_MEAS_STATUS_ADDR 0x1D // measuring<5>, new_data<0>

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
  dev->calib.press_calib.par_p6 =
      (uint8_t)buffer_a[0x99 - BME680_BLOCK_A_START];
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

bme680_status_t bme680_set_hum_oversampling(bme680_t *dev, uint8_t osrs_h) {
  if (dev == NULL) {
    return BME680_NULL_PTR;
  }
  uint8_t ctrl_hum = osrs_h & 0b00000111;
  uint8_t bme_hum_write_result = dev->i2c_write(
      dev->ctx, dev->dev_addr, BME680_CTRL_HUM_ADDR, &ctrl_hum, 1);
  if (bme_hum_write_result < 0) {
    return BME680_I2C_ERROR;
  }
  return BME680_OK;
}

bme680_status_t bme680_set_config_register(bme680_t *dev, uint8_t config) {
  if (dev == NULL) {
    return BME680_NULL_PTR;
  }
  uint8_t config_value = config & 0b00011100;
  uint8_t bme_config_write_result = dev->i2c_write(
      dev->ctx, dev->dev_addr, BME680_IIR_ADDR, &config_value, 1);
  if (bme_config_write_result < 0) {
    return BME680_I2C_ERROR;
  }
  return BME680_OK;
}

bme680_status_t bme680_set_ctrl_meas_register(bme680_t *dev,
                                              uint8_t ctrl_meas) {
  if (dev == NULL) {
    return BME680_NULL_PTR;
  }
  uint8_t bme_ctrl_meas_write_result = dev->i2c_write(
      dev->ctx, dev->dev_addr, BME680_CTRL_MEAS_ADDR, &ctrl_meas, 1);
  if (bme_ctrl_meas_write_result < 0) {
    return BME680_I2C_ERROR;
  }
  return BME680_OK;
}

bme680_status_t bme680_set_parameters(bme680_t *dev, uint8_t config,
                                      uint8_t hum_oversampling,
                                      uint8_t ctrl_meas) {
  if (dev == NULL) {
    return BME680_NULL_PTR;
  }
  if (bme680_set_hum_oversampling(dev, hum_oversampling) != BME680_OK) {
    return BME680_I2C_ERROR;
  }
  if (bme680_set_config_register(dev, config) != BME680_OK) {
    return BME680_I2C_ERROR;
  }
  if (bme680_set_ctrl_meas_register(dev, ctrl_meas) != BME680_OK) {
    return BME680_I2C_ERROR;
  }
  return BME680_OK;
}

bme680_status_t bme680_init(bme680_t *dev) {
  if (dev == NULL) {
    return BME680_NULL_PTR;
  }
  uint8_t ctrl_meas =
      0b00100101; // osrs_t=001 (x1), osrs_p=001 (x1), mode=01 (forced)
  return bme680_set_parameters(dev, 0, 1, ctrl_meas);
}

bme680_status_t bme680_get_measurement_status(bme680_t *dev, uint8_t *measuring,
                                              uint8_t *new_data) {
  if (dev == NULL || measuring == NULL || new_data == NULL) {
    return BME680_NULL_PTR;
  }
  uint8_t meas_status;
  int result = dev->i2c_read(dev->ctx, dev->dev_addr, BME680_MEAS_STATUS_ADDR,
                             &meas_status, 1);
  if (result < 0) {
    return BME680_I2C_ERROR;
  }
  *measuring = meas_status & 0b00100000;
  *new_data = meas_status & 0b1000000;
  return BME680_OK;
}

bool bme680_is_measuring(bme680_t *dev) {
  if (dev == NULL) {
    return false;
  }
  uint8_t measuring, new_data;
  if (bme680_get_measurement_status(dev, &measuring, &new_data) != BME680_OK) {
    return false;
  }
  return measuring == 0b00100000;
}

bool bme680_is_new_data_available(bme680_t *dev) {
  if (dev == NULL) {
    return false;
  }
  uint8_t measuring, new_data;
  if (bme680_get_measurement_status(dev, &measuring, &new_data) != BME680_OK) {
    return false;
  }
  return new_data == 0b0000001;
}

bme680_status_t bme680_start_forced_measurement(bme680_t *dev) {
  if (dev == NULL) {
    return BME680_NULL_PTR;
  }
  uint8_t ctrl_meas =
      0b00100101; // osrs_t=001 (x1), osrs_p=001 (x1), mode=01 (forced)
  return bme680_set_ctrl_meas_register(dev, ctrl_meas);
}

bme680_status_t bme680_read_measurement_forced(bme680_t *dev,
                                               float *temperature,
                                               float *pressure,
                                               float *humidity) {
  if (dev == NULL || temperature == NULL || pressure == NULL ||
      humidity == NULL) {
    return BME680_NULL_PTR;
  }
  uint8_t meas_status;
  uint8_t pressure_msb, pressure_lsb, pressure_xlsb;
  uint8_t temperature_msb, temperature_lsb, temperature_xlsb;
  uint8_t humidity_msb, humidity_lsb;

  bme680_status_t status = bme680_start_forced_measurement(dev);
  if (status != BME680_OK) {
    return status;
  }

  bme680_start_forced_measurement(dev);
  uint8_t check;
  dev->i2c_read(dev->ctx, dev->dev_addr, BME680_CTRL_MEAS_ADDR, &check, 1);

  uint8_t raw;
  dev->i2c_read(dev->ctx, dev->dev_addr, BME680_MEAS_STATUS_ADDR, &raw, 1);

  int i = 0;
  while (bme680_is_new_data_available(dev) && i < 100) {
    i++;
  }
  if (i >= 100) {
    return BME680_TIMEOUT;
  }

  dev->i2c_read(dev->ctx, dev->dev_addr, BME680_PRESSURE_MSB_ADDR,
                &pressure_msb, 1);
  dev->i2c_read(dev->ctx, dev->dev_addr, BME680_PRESSURE_LSB_ADDR,
                &pressure_lsb, 1);
  dev->i2c_read(dev->ctx, dev->dev_addr, BME680_PRESSURE_XLSB_ADDR,
                &pressure_xlsb, 1);
  dev->i2c_read(dev->ctx, dev->dev_addr, BME680_TEMPERATURE_MSB_ADDR,
                &temperature_msb, 1);
  dev->i2c_read(dev->ctx, dev->dev_addr, BME680_TEMPERATURE_LSB_ADDR,
                &temperature_lsb, 1);
  dev->i2c_read(dev->ctx, dev->dev_addr, BME680_TEMPERATURE_XLSB_ADDR,
                &temperature_xlsb, 1);
  dev->i2c_read(dev->ctx, dev->dev_addr, BME680_HUMIDITY_MSB_ADDR,
                &humidity_msb, 1);
  dev->i2c_read(dev->ctx, dev->dev_addr, BME680_HUMIDITY_LSB_ADDR,
                &humidity_lsb, 1);

  uint32_t temperature_adc, pressure_adc, humidity_adc;

  temperature_adc = ((uint32_t)temperature_msb << 12) |
                    ((uint32_t)temperature_lsb << 4) |
                    ((uint32_t)(temperature_xlsb >> 4));
  pressure_adc = ((uint32_t)pressure_msb << 12) |
                 ((uint32_t)pressure_lsb << 4) |
                 ((uint32_t)(pressure_xlsb >> 4));
  humidity_adc = ((uint32_t)humidity_msb << 8) | ((uint32_t)humidity_lsb);

  *temperature = bme680_compensate_temperature(dev, temperature_adc);
  *pressure = bme680_compensate_pressure(dev, pressure_adc);
  *humidity = bme680_compensate_humidity(dev, humidity_adc);

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
  bme680_init(&instance);
  instance_used = 1;
  return &instance;
}