#ifndef SHT40_H
#define SHT40_H

#include <stddef.h>
#include <stdint.h>

typedef struct sht40 sht40_t;

typedef enum {
  SHT40_OK,
  SHT40_I2C_ERROR,
  SHT40_NULL_PTR,
  SHT40_CRC_ERROR,
} sht40_status_t;

typedef enum {
  SHT40_PRECISION_HIGH = 0xFD,
  SHT40_PRECISION_MEDIUM = 0xF6,
  SHT40_PRECISION_LOW = 0xE0,
} sht40_precision_t;

typedef enum {
  SHT40_HEATER_200MW_1S = 0x39,
  SHT40_HEATER_200MW_100MS = 0x32,
  SHT40_HEATER_110MW_1S = 0x2F,
  SHT40_HEATER_110MW_100MS = 0x24,
  SHT40_HEATER_20MW_1S = 0x1E,
  SHT40_HEATER_20MW_100MS = 0x15,
} sht40_heater_t;

typedef int (*sht40_i2c_write_fn)(void *ctx, uint8_t dev_addr,
                                  const uint8_t *data, size_t length);
typedef int (*sht40_i2c_read_fn)(void *ctx, uint8_t dev_addr, uint8_t *data,
                                 size_t length);
typedef void (*sht40_delay_ms_fn)(void *ctx, uint32_t ms);

sht40_t *sht40_create(void *ctx, uint8_t dev_addr, sht40_i2c_write_fn i2c_write,
                      sht40_i2c_read_fn i2c_read, sht40_delay_ms_fn delay_ms);

sht40_status_t sht40_soft_reset(sht40_t *dev);
sht40_status_t sht40_read_serial(sht40_t *dev, uint32_t *serial);

sht40_status_t sht40_start_measurement(sht40_t *dev,
                                       sht40_precision_t precision);
sht40_status_t sht40_start_heater(sht40_t *dev, sht40_heater_t heater);
uint32_t sht40_measurement_wait_ms(sht40_precision_t precision);
uint32_t sht40_heater_wait_ms(sht40_heater_t heater);
sht40_status_t sht40_read_measurement(sht40_t *dev, float *temperature,
                                      float *humidity);

sht40_status_t sht40_measure(sht40_t *dev, sht40_precision_t precision,
                             float *temperature, float *humidity);

#endif // SHT40_H