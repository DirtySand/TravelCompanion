#ifndef QMC5883L_H
#define QMC5883L_H

#include <stdbool.h>
#include <stdint.h>

#define QMC5883L_CHIP_ID 0xFF

typedef struct qmc5883l qmc5883l_t;

typedef enum {
  QMC5883L_OK,
  QMC5883L_I2C_ERROR,
  QMC5883L_NULL_PTR,
  QMC5883L_NOT_READY,
  QMC5883L_OVERFLOW,
} qmc5883l_status_t;

typedef enum {
  QMC5883L_ODR_10HZ = 0,
  QMC5883L_ODR_50HZ = 1,
  QMC5883L_ODR_100HZ = 2,
  QMC5883L_ODR_200HZ = 3,
} qmc5883l_odr_t;

typedef enum {
  QMC5883L_RANGE_2G = 0,
  QMC5883L_RANGE_8G = 1,
} qmc5883l_range_t;

typedef enum {
  QMC5883L_OSR_512 = 0,
  QMC5883L_OSR_256 = 1,
  QMC5883L_OSR_128 = 2,
  QMC5883L_OSR_64 = 3,
} qmc5883l_osr_t;

typedef struct {
  qmc5883l_odr_t odr;
  qmc5883l_range_t range;
  qmc5883l_osr_t osr;
} qmc5883l_config_t;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} qmc5883l_raw_t;

typedef int (*qmc5883l_i2c_read_fn)(void *ctx, uint8_t dev_addr,
                                    uint8_t reg_addr, uint8_t *data,
                                    uint8_t length);
typedef int (*qmc5883l_i2c_write_fn)(void *ctx, uint8_t dev_addr,
                                     uint8_t reg_addr, const uint8_t *data,
                                     uint8_t length);
typedef void (*qmc5883l_delay_ms_fn)(void *ctx, uint32_t ms);

qmc5883l_t *qmc5883l_create(void *ctx, uint8_t dev_addr,
                            qmc5883l_i2c_read_fn i2c_read,
                            qmc5883l_i2c_write_fn i2c_write,
                            qmc5883l_delay_ms_fn delay_ms);

qmc5883l_status_t qmc5883l_read_chip_id(qmc5883l_t *dev, uint8_t *chip_id);

qmc5883l_status_t qmc5883l_init(qmc5883l_t *dev,
                                const qmc5883l_config_t *config);

qmc5883l_status_t qmc5883l_start(qmc5883l_t *dev);
qmc5883l_status_t qmc5883l_standby(qmc5883l_t *dev);

qmc5883l_status_t qmc5883l_read_raw(qmc5883l_t *dev, qmc5883l_raw_t *raw);

float qmc5883l_lsb_per_ut(const qmc5883l_t *dev);

#endif // QMC5883L_H
