#include "qmc5883l.h"
#include <stddef.h>

#define QMC5883L_REG_DATA_X_LSB 0x00
#define QMC5883L_REG_STATUS 0x06
#define QMC5883L_REG_CONTROL_1 0x09
#define QMC5883L_REG_CONTROL_2 0x0A
#define QMC5883L_REG_SET_RESET_PERIOD 0x0B
#define QMC5883L_REG_CHIP_ID 0x0D
#define QMC5883L_DATA_LENGTH 6
#define QMC5883L_STATUS_DRDY (1u << 0)
#define QMC5883L_STATUS_OVL (1u << 1)
#define QMC5883L_OSR_SHIFT 6
#define QMC5883L_RNG_SHIFT 4
#define QMC5883L_ODR_SHIFT 2
#define QMC5883L_FIELD_MASK 0x03u
#define QMC5883L_MODE_MASK 0x03u
#define QMC5883L_MODE_STANDBY 0x00u
#define QMC5883L_MODE_CONTINUOUS 0x01u
#define QMC5883L_SOFT_RESET (1u << 7)
#define QMC5883L_INT_DISABLE (1u << 0)
#define QMC5883L_SET_RESET_PERIOD 0x01
#define QMC5883L_RESET_WAIT_MS 10
#define QMC5883L_LSB_PER_UT_2G 120.0f
#define QMC5883L_LSB_PER_UT_8G 30.0f

struct qmc5883l {
  void *ctx;
  uint8_t dev_addr;
  qmc5883l_i2c_read_fn i2c_read;
  qmc5883l_i2c_write_fn i2c_write;
  qmc5883l_delay_ms_fn delay_ms;
  uint8_t control_1;
  qmc5883l_range_t range;
};

static qmc5883l_t instance;
static int instance_used = 0;

static qmc5883l_status_t read_registers(qmc5883l_t *dev, uint8_t reg,
                                        uint8_t *data, uint8_t length) {
  int result = dev->i2c_read(dev->ctx, dev->dev_addr, reg, data, length);
  if (result != (int)length) {
    return QMC5883L_I2C_ERROR;
  }
  return QMC5883L_OK;
}

static qmc5883l_status_t write_register(qmc5883l_t *dev, uint8_t reg,
                                        uint8_t value) {
  int result = dev->i2c_write(dev->ctx, dev->dev_addr, reg, &value, 1);
  if (result != 2) {
    return QMC5883L_I2C_ERROR;
  }
  return QMC5883L_OK;
}

static int16_t to_int16(uint8_t lsb, uint8_t msb) {
  return (int16_t)((uint16_t)lsb | ((uint16_t)msb << 8));
}

static uint8_t control_1_value(const qmc5883l_config_t *config) {
  return (
      uint8_t)(((config->osr & QMC5883L_FIELD_MASK) << QMC5883L_OSR_SHIFT) |
               ((config->range & QMC5883L_FIELD_MASK) << QMC5883L_RNG_SHIFT) |
               ((config->odr & QMC5883L_FIELD_MASK) << QMC5883L_ODR_SHIFT));
}

static qmc5883l_status_t set_mode(qmc5883l_t *dev, uint8_t mode) {
  return write_register(
      dev, QMC5883L_REG_CONTROL_1,
      (uint8_t)(dev->control_1 | (mode & QMC5883L_MODE_MASK)));
}

qmc5883l_t *qmc5883l_create(void *ctx, uint8_t dev_addr,
                            qmc5883l_i2c_read_fn i2c_read,
                            qmc5883l_i2c_write_fn i2c_write,
                            qmc5883l_delay_ms_fn delay_ms) {
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
  instance.control_1 = 0;
  instance.range = QMC5883L_RANGE_2G;
  instance_used = 1;
  return &instance;
}

qmc5883l_status_t qmc5883l_read_chip_id(qmc5883l_t *dev, uint8_t *chip_id) {
  if (dev == NULL || chip_id == NULL) {
    return QMC5883L_NULL_PTR;
  }
  return read_registers(dev, QMC5883L_REG_CHIP_ID, chip_id, 1);
}

qmc5883l_status_t qmc5883l_init(qmc5883l_t *dev,
                                const qmc5883l_config_t *config) {
  if (dev == NULL || config == NULL) {
    return QMC5883L_NULL_PTR;
  }
  qmc5883l_status_t status =
      write_register(dev, QMC5883L_REG_CONTROL_2, QMC5883L_SOFT_RESET);
  if (status != QMC5883L_OK) {
    return status;
  }
  dev->delay_ms(dev->ctx, QMC5883L_RESET_WAIT_MS);

  status = write_register(dev, QMC5883L_REG_CONTROL_2, QMC5883L_INT_DISABLE);
  if (status != QMC5883L_OK) {
    return status;
  }
  status = write_register(dev, QMC5883L_REG_SET_RESET_PERIOD,
                          QMC5883L_SET_RESET_PERIOD);
  if (status != QMC5883L_OK) {
    return status;
  }
  dev->control_1 = control_1_value(config);
  dev->range = config->range;
  return set_mode(dev, QMC5883L_MODE_STANDBY);
}

qmc5883l_status_t qmc5883l_start(qmc5883l_t *dev) {
  if (dev == NULL) {
    return QMC5883L_NULL_PTR;
  }
  return set_mode(dev, QMC5883L_MODE_CONTINUOUS);
}

qmc5883l_status_t qmc5883l_standby(qmc5883l_t *dev) {
  if (dev == NULL) {
    return QMC5883L_NULL_PTR;
  }
  return set_mode(dev, QMC5883L_MODE_STANDBY);
}

qmc5883l_status_t qmc5883l_read_raw(qmc5883l_t *dev, qmc5883l_raw_t *raw) {
  if (dev == NULL || raw == NULL) {
    return QMC5883L_NULL_PTR;
  }
  uint8_t status_reg = 0;
  qmc5883l_status_t status =
      read_registers(dev, QMC5883L_REG_STATUS, &status_reg, 1);
  if (status != QMC5883L_OK) {
    return status;
  }
  if (!(status_reg & QMC5883L_STATUS_DRDY)) {
    return QMC5883L_NOT_READY;
  }
  uint8_t data[QMC5883L_DATA_LENGTH];
  status =
      read_registers(dev, QMC5883L_REG_DATA_X_LSB, data, QMC5883L_DATA_LENGTH);
  if (status != QMC5883L_OK) {
    return status;
  }
  raw->x = to_int16(data[0], data[1]);
  raw->y = to_int16(data[2], data[3]);
  raw->z = to_int16(data[4], data[5]);
  return (status_reg & QMC5883L_STATUS_OVL) ? QMC5883L_OVERFLOW : QMC5883L_OK;
}

float qmc5883l_lsb_per_ut(const qmc5883l_t *dev) {
  if (dev != NULL && dev->range == QMC5883L_RANGE_8G) {
    return QMC5883L_LSB_PER_UT_8G;
  }
  return QMC5883L_LSB_PER_UT_2G;
}
