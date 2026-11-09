#ifndef ICM20649_H
#define ICM20649_H

#include <stdbool.h>
#include <stdint.h>

#define ICM20649_CHIP_ID 0xE1
#define ICM20649_FIFO_ACCEL (1u << 4)
#define ICM20649_FIFO_GYRO_Z (1u << 3)
#define ICM20649_FIFO_GYRO_Y (1u << 2)
#define ICM20649_FIFO_GYRO_X (1u << 1)
#define ICM20649_FIFO_TEMP (1u << 0)

typedef struct icm20649 icm20649_t;

typedef enum {
  ICM20649_OK,
  ICM20649_I2C_ERROR,
  ICM20649_NULL_PTR,
  ICM20649_WRONG_CHIP_ID,
  ICM20649_RESET_TIMEOUT,
} icm20649_status_t;

typedef enum {
  ICM20649_ACCEL_4G = 0,
  ICM20649_ACCEL_8G = 1,
  ICM20649_ACCEL_16G = 2,
  ICM20649_ACCEL_30G = 3,
} icm20649_accel_range_t;

typedef enum {
  ICM20649_GYRO_500DPS = 0,
  ICM20649_GYRO_1000DPS = 1,
  ICM20649_GYRO_2000DPS = 2,
  ICM20649_GYRO_4000DPS = 3,
} icm20649_gyro_range_t;

typedef struct {
  icm20649_accel_range_t accel_range;
  uint16_t accel_rate_div;
  uint8_t accel_dlpf;
  icm20649_gyro_range_t gyro_range;
  uint8_t gyro_rate_div;
  uint8_t gyro_dlpf;
} icm20649_config_t;

typedef struct {
  int16_t accel[3];
  int16_t gyro[3];
  int16_t temperature;
} icm20649_raw_t;

typedef int (*icm20649_i2c_read_fn)(void *ctx, uint8_t dev_addr,
                                    uint8_t reg_addr, uint8_t *data,
                                    uint8_t length);
typedef int (*icm20649_i2c_write_fn)(void *ctx, uint8_t dev_addr,
                                     uint8_t reg_addr, const uint8_t *data,
                                     uint8_t length);
typedef void (*icm20649_delay_ms_fn)(void *ctx, uint32_t ms);

icm20649_t *icm20649_create(void *ctx, uint8_t dev_addr,
                            icm20649_i2c_read_fn i2c_read,
                            icm20649_i2c_write_fn i2c_write,
                            icm20649_delay_ms_fn delay_ms);

icm20649_status_t icm20649_read_chip_id(icm20649_t *dev, uint8_t *chip_id);

icm20649_status_t icm20649_init(icm20649_t *dev,
                                const icm20649_config_t *config);

icm20649_status_t icm20649_set_sleep(icm20649_t *dev, bool sleep);

icm20649_status_t icm20649_read_raw(icm20649_t *dev, icm20649_raw_t *raw);

icm20649_status_t icm20649_set_rate_dividers(icm20649_t *dev,
                                             uint16_t accel_rate_div,
                                             uint8_t gyro_rate_div);

icm20649_status_t icm20649_fifo_start(icm20649_t *dev, uint8_t sources,
                                      bool stop_when_full);
icm20649_status_t icm20649_fifo_stop(icm20649_t *dev);
icm20649_status_t icm20649_fifo_count(icm20649_t *dev, uint16_t *count);
icm20649_status_t icm20649_fifo_overflowed(icm20649_t *dev, bool *overflow);

float icm20649_accel_lsb_per_g(const icm20649_t *dev);
float icm20649_gyro_lsb_per_dps(const icm20649_t *dev);
float icm20649_temperature_c(int16_t raw);

#endif // ICM20649_H
