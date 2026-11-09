#include "icm20649.h"
#include <stddef.h>

#define REG_WHO_AM_I 0x00
#define REG_USER_CTRL 0x03
#define REG_PWR_MGMT_1 0x06
#define REG_INT_STATUS_2 0x1B
#define REG_ACCEL_XOUT_H 0x2D
#define REG_FIFO_EN_2 0x67
#define REG_FIFO_RST 0x68
#define REG_FIFO_MODE 0x69
#define REG_FIFO_COUNTH 0x70
#define REG_BANK_SEL 0x7F
#define REG_GYRO_SMPLRT_DIV 0x00
#define REG_GYRO_CONFIG_1 0x01
#define REG_ACCEL_SMPLRT_DIV_1 0x10
#define REG_ACCEL_SMPLRT_DIV_2 0x11
#define REG_ACCEL_CONFIG 0x14
#define BANK_0 0
#define BANK_2 2
#define BANK_UNKNOWN 0xFF
#define BANK_SHIFT 4
#define USER_CTRL_FIFO_EN (1u << 6)
#define PWR_MGMT_1_DEVICE_RESET (1u << 7)
#define PWR_MGMT_1_SLEEP (1u << 6)
#define PWR_MGMT_1_CLKSEL_AUTO 0x01u
#define CONFIG_DLPF_SHIFT 3
#define CONFIG_FS_SHIFT 1
#define CONFIG_FCHOICE 0x01u
#define CONFIG_DLPF_MASK 0x07u
#define CONFIG_FS_MASK 0x03u
#define FIFO_RST_ALL 0x1Fu
#define FIFO_MODE_SNAPSHOT 0x1Fu
#define FIFO_MODE_STREAM 0x00u
#define FIFO_COUNT_HIGH_MASK 0x1Fu
#define INT_STATUS_2_FIFO_OVERFLOW 0x1Fu
#define ACCEL_MAX_RATE_DIV 0x0FFFu
#define RAW_DATA_LENGTH 14
#define RESET_POLL_MS 10
#define RESET_POLL_COUNT 10
#define GYRO_STARTUP_MS 35
#define TEMPERATURE_LSB_PER_C 333.87f
#define TEMPERATURE_ROOM_C 21.0f

static const float accel_lsb_per_g[] = {8192.0f, 4096.0f, 2048.0f, 1024.0f};
static const float gyro_lsb_per_dps[] = {65.5f, 32.8f, 16.4f, 8.2f};

struct icm20649 {
  void *ctx;
  uint8_t dev_addr;
  icm20649_i2c_read_fn i2c_read;
  icm20649_i2c_write_fn i2c_write;
  icm20649_delay_ms_fn delay_ms;
  uint8_t bank;
  icm20649_accel_range_t accel_range;
  icm20649_gyro_range_t gyro_range;
};

static icm20649_t instance;
static int instance_used = 0;

static icm20649_status_t write_raw(icm20649_t *dev, uint8_t reg,
                                   uint8_t value) {
  int result = dev->i2c_write(dev->ctx, dev->dev_addr, reg, &value, 1);
  return (result == 2) ? ICM20649_OK : ICM20649_I2C_ERROR;
}

static icm20649_status_t select_bank(icm20649_t *dev, uint8_t bank) {
  if (dev->bank == bank) {
    return ICM20649_OK;
  }
  icm20649_status_t status =
      write_raw(dev, REG_BANK_SEL, (uint8_t)(bank << BANK_SHIFT));
  dev->bank = (status == ICM20649_OK) ? bank : BANK_UNKNOWN;
  return status;
}

static icm20649_status_t read_registers(icm20649_t *dev, uint8_t bank,
                                        uint8_t reg, uint8_t *data,
                                        uint8_t length) {
  icm20649_status_t status = select_bank(dev, bank);
  if (status != ICM20649_OK) {
    return status;
  }
  int result = dev->i2c_read(dev->ctx, dev->dev_addr, reg, data, length);
  return (result == (int)length) ? ICM20649_OK : ICM20649_I2C_ERROR;
}

static icm20649_status_t write_register(icm20649_t *dev, uint8_t bank,
                                        uint8_t reg, uint8_t value) {
  icm20649_status_t status = select_bank(dev, bank);
  if (status != ICM20649_OK) {
    return status;
  }
  return write_raw(dev, reg, value);
}

static int16_t to_int16(uint8_t msb, uint8_t lsb) {
  return (int16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
}

static uint8_t config_value(uint8_t dlpf, uint8_t range) {
  return (uint8_t)(((dlpf & CONFIG_DLPF_MASK) << CONFIG_DLPF_SHIFT) |
                   ((range & CONFIG_FS_MASK) << CONFIG_FS_SHIFT) |
                   CONFIG_FCHOICE);
}

static icm20649_status_t reset(icm20649_t *dev) {
  icm20649_status_t status =
      write_register(dev, BANK_0, REG_PWR_MGMT_1, PWR_MGMT_1_DEVICE_RESET);
  if (status != ICM20649_OK) {
    return status;
  }
  dev->bank = BANK_0;
  for (uint8_t i = 0; i < RESET_POLL_COUNT; i++) {
    dev->delay_ms(dev->ctx, RESET_POLL_MS);
    uint8_t value = 0;
    int result = dev->i2c_read(dev->ctx, dev->dev_addr, REG_PWR_MGMT_1,
                               &value, 1);
    if (result == 1 && !(value & PWR_MGMT_1_DEVICE_RESET)) {
      return ICM20649_OK;
    }
  }
  return ICM20649_RESET_TIMEOUT;
}

icm20649_t *icm20649_create(void *ctx, uint8_t dev_addr,
                            icm20649_i2c_read_fn i2c_read,
                            icm20649_i2c_write_fn i2c_write,
                            icm20649_delay_ms_fn delay_ms) {
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
  instance.bank = BANK_UNKNOWN;
  instance.accel_range = ICM20649_ACCEL_4G;
  instance.gyro_range = ICM20649_GYRO_500DPS;
  instance_used = 1;
  return &instance;
}

icm20649_status_t icm20649_read_chip_id(icm20649_t *dev, uint8_t *chip_id) {
  if (dev == NULL || chip_id == NULL) {
    return ICM20649_NULL_PTR;
  }
  return read_registers(dev, BANK_0, REG_WHO_AM_I, chip_id, 1);
}

icm20649_status_t icm20649_set_rate_dividers(icm20649_t *dev,
                                             uint16_t accel_rate_div,
                                             uint8_t gyro_rate_div) {
  if (dev == NULL) {
    return ICM20649_NULL_PTR;
  }
  accel_rate_div &= ACCEL_MAX_RATE_DIV;
  icm20649_status_t status = write_register(
      dev, BANK_2, REG_ACCEL_SMPLRT_DIV_1, (uint8_t)(accel_rate_div >> 8));
  if (status == ICM20649_OK) {
    status = write_register(dev, BANK_2, REG_ACCEL_SMPLRT_DIV_2,
                            (uint8_t)(accel_rate_div & 0xFFu));
  }
  if (status == ICM20649_OK) {
    status = write_register(dev, BANK_2, REG_GYRO_SMPLRT_DIV, gyro_rate_div);
  }
  if (status == ICM20649_OK) {
    status = select_bank(dev, BANK_0);
  }
  return status;
}

icm20649_status_t icm20649_init(icm20649_t *dev,
                                const icm20649_config_t *config) {
  if (dev == NULL || config == NULL) {
    return ICM20649_NULL_PTR;
  }
  uint8_t chip_id = 0;
  icm20649_status_t status = icm20649_read_chip_id(dev, &chip_id);
  if (status != ICM20649_OK) {
    return status;
  }
  if (chip_id != ICM20649_CHIP_ID) {
    return ICM20649_WRONG_CHIP_ID;
  }
  status = reset(dev);
  if (status != ICM20649_OK) {
    return status;
  }
  status = write_register(dev, BANK_0, REG_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_AUTO);
  if (status != ICM20649_OK) {
    return status;
  }

  status = write_register(dev, BANK_2, REG_GYRO_CONFIG_1,
                          config_value(config->gyro_dlpf, config->gyro_range));
  if (status == ICM20649_OK) {
    status =
        write_register(dev, BANK_2, REG_ACCEL_CONFIG,
                       config_value(config->accel_dlpf, config->accel_range));
  }
  if (status == ICM20649_OK) {
    status = icm20649_set_rate_dividers(dev, config->accel_rate_div,
                                        config->gyro_rate_div);
  }
  if (status != ICM20649_OK) {
    return status;
  }
  dev->accel_range = config->accel_range;
  dev->gyro_range = config->gyro_range;
  dev->delay_ms(dev->ctx, GYRO_STARTUP_MS);
  return ICM20649_OK;
}

icm20649_status_t icm20649_set_sleep(icm20649_t *dev, bool sleep) {
  if (dev == NULL) {
    return ICM20649_NULL_PTR;
  }
  uint8_t value = (uint8_t)(PWR_MGMT_1_CLKSEL_AUTO |
                            (sleep ? PWR_MGMT_1_SLEEP : 0u));
  icm20649_status_t status = write_register(dev, BANK_0, REG_PWR_MGMT_1, value);
  if (status == ICM20649_OK && !sleep) {
    dev->delay_ms(dev->ctx, GYRO_STARTUP_MS);
  }
  return status;
}

icm20649_status_t icm20649_read_raw(icm20649_t *dev, icm20649_raw_t *raw) {
  if (dev == NULL || raw == NULL) {
    return ICM20649_NULL_PTR;
  }
  uint8_t data[RAW_DATA_LENGTH];
  icm20649_status_t status =
      read_registers(dev, BANK_0, REG_ACCEL_XOUT_H, data, RAW_DATA_LENGTH);
  if (status != ICM20649_OK) {
    return status;
  }
  for (uint8_t axis = 0; axis < 3; axis++) {
    raw->accel[axis] = to_int16(data[2 * axis], data[2 * axis + 1]);
    raw->gyro[axis] = to_int16(data[6 + 2 * axis], data[7 + 2 * axis]);
  }
  raw->temperature = to_int16(data[12], data[13]);
  return ICM20649_OK;
}

icm20649_status_t icm20649_fifo_start(icm20649_t *dev, uint8_t sources,
                                      bool stop_when_full) {
  if (dev == NULL) {
    return ICM20649_NULL_PTR;
  }
  icm20649_status_t status = write_register(dev, BANK_0, REG_FIFO_EN_2, 0);
  if (status == ICM20649_OK) {
    status = write_register(dev, BANK_0, REG_FIFO_MODE,
                            stop_when_full ? FIFO_MODE_SNAPSHOT
                                           : FIFO_MODE_STREAM);
  }
  uint8_t user_ctrl = 0;
  if (status == ICM20649_OK) {
    status = read_registers(dev, BANK_0, REG_USER_CTRL, &user_ctrl, 1);
  }
  if (status == ICM20649_OK) {
    status = write_register(dev, BANK_0, REG_USER_CTRL,
                            (uint8_t)(user_ctrl | USER_CTRL_FIFO_EN));
  }
  if (status == ICM20649_OK) {
    status = write_register(dev, BANK_0, REG_FIFO_RST, FIFO_RST_ALL);
  }
  if (status == ICM20649_OK) {
    status = write_register(dev, BANK_0, REG_FIFO_RST, 0);
  }
  if (status == ICM20649_OK) {
    bool ignored;
    status = icm20649_fifo_overflowed(dev, &ignored); // clear old flags
  }
  if (status == ICM20649_OK) {
    status = write_register(dev, BANK_0, REG_FIFO_EN_2, sources);
  }
  return status;
}

icm20649_status_t icm20649_fifo_stop(icm20649_t *dev) {
  if (dev == NULL) {
    return ICM20649_NULL_PTR;
  }
  icm20649_status_t status = write_register(dev, BANK_0, REG_FIFO_EN_2, 0);
  uint8_t user_ctrl = 0;
  if (status == ICM20649_OK) {
    status = read_registers(dev, BANK_0, REG_USER_CTRL, &user_ctrl, 1);
  }
  if (status == ICM20649_OK) {
    status = write_register(dev, BANK_0, REG_USER_CTRL,
                            (uint8_t)(user_ctrl & ~USER_CTRL_FIFO_EN));
  }
  if (status == ICM20649_OK) {
    status = write_register(dev, BANK_0, REG_FIFO_RST, FIFO_RST_ALL);
  }
  if (status == ICM20649_OK) {
    status = write_register(dev, BANK_0, REG_FIFO_RST, 0);
  }
  return status;
}

icm20649_status_t icm20649_fifo_count(icm20649_t *dev, uint16_t *count) {
  if (dev == NULL || count == NULL) {
    return ICM20649_NULL_PTR;
  }
  uint8_t data[2];
  icm20649_status_t status =
      read_registers(dev, BANK_0, REG_FIFO_COUNTH, data, 2);
  if (status != ICM20649_OK) {
    return status;
  }
  *count = (uint16_t)(((uint16_t)(data[0] & FIFO_COUNT_HIGH_MASK) << 8) |
                      data[1]);
  return ICM20649_OK;
}

icm20649_status_t icm20649_fifo_overflowed(icm20649_t *dev, bool *overflow) {
  if (dev == NULL || overflow == NULL) {
    return ICM20649_NULL_PTR;
  }
  uint8_t value = 0;
  icm20649_status_t status =
      read_registers(dev, BANK_0, REG_INT_STATUS_2, &value, 1);
  if (status != ICM20649_OK) {
    return status;
  }
  *overflow = (value & INT_STATUS_2_FIFO_OVERFLOW) != 0;
  return ICM20649_OK;
}

float icm20649_accel_lsb_per_g(const icm20649_t *dev) {
  return accel_lsb_per_g[(dev != NULL) ? (dev->accel_range & 3u) : 0];
}

float icm20649_gyro_lsb_per_dps(const icm20649_t *dev) {
  return gyro_lsb_per_dps[(dev != NULL) ? (dev->gyro_range & 3u) : 0];
}

float icm20649_temperature_c(int16_t raw) {
  return (float)raw / TEMPERATURE_LSB_PER_C + TEMPERATURE_ROOM_C;
}
