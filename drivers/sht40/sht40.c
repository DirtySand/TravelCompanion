#include "sht40.h"
#include <stddef.h>

#define SHT40_CMD_READ_SERIAL 0x89
#define SHT40_CMD_SOFT_RESET 0x94

#define SHT40_RESPONSE_LENGTH 6
#define SHT40_WORD_LENGTH 2

#define SHT40_CRC8_POLYNOMIAL 0x31
#define SHT40_CRC8_INIT 0xFF

#define SHT40_WAIT_HIGH_MS 10
#define SHT40_WAIT_MEDIUM_MS 5
#define SHT40_WAIT_LOW_MS 2
#define SHT40_WAIT_HEATER_1S_MS 1100
#define SHT40_WAIT_HEATER_100MS_MS 110
#define SHT40_WAIT_SERIAL_MS 10
#define SHT40_WAIT_SOFT_RESET_MS 1

#define SHT40_TICKS_FULL_SCALE 65535.0f

struct sht40 {
  void *ctx;
  uint8_t dev_addr;
  sht40_i2c_write_fn i2c_write;
  sht40_i2c_read_fn i2c_read;
  sht40_delay_ms_fn delay_ms;
};

static sht40_t instance;
static int instance_used = 0;

static uint8_t sht40_crc8(const uint8_t *data, uint8_t length) {
  uint8_t crc = SHT40_CRC8_INIT;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ SHT40_CRC8_POLYNOMIAL);
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}

static sht40_status_t sht40_send_command(sht40_t *dev, uint8_t command) {
  int result = dev->i2c_write(dev->ctx, dev->dev_addr, &command, 1);
  if (result != 1) {
    return SHT40_I2C_ERROR;
  }
  return SHT40_OK;
}

static sht40_status_t sht40_read_words(sht40_t *dev, uint16_t *word0,
                                       uint16_t *word1) {
  uint8_t buffer[SHT40_RESPONSE_LENGTH];
  int result =
      dev->i2c_read(dev->ctx, dev->dev_addr, buffer, SHT40_RESPONSE_LENGTH);
  if (result != SHT40_RESPONSE_LENGTH) {
    return SHT40_I2C_ERROR;
  }
  if (sht40_crc8(&buffer[0], SHT40_WORD_LENGTH) != buffer[2] ||
      sht40_crc8(&buffer[3], SHT40_WORD_LENGTH) != buffer[5]) {
    return SHT40_CRC_ERROR;
  }
  *word0 = (uint16_t)((buffer[0] << 8) | buffer[1]);
  *word1 = (uint16_t)((buffer[3] << 8) | buffer[4]);
  return SHT40_OK;
}

static float sht40_ticks_to_celsius(uint16_t ticks) {
  return -45.0f + 175.0f * (float)ticks / SHT40_TICKS_FULL_SCALE;
}

static float sht40_ticks_to_humidity(uint16_t ticks) {
  float humidity = -6.0f + 125.0f * (float)ticks / SHT40_TICKS_FULL_SCALE;
  if (humidity > 100.0f) {
    humidity = 100.0f;
  } else if (humidity < 0.0f) {
    humidity = 0.0f;
  }
  return humidity;
}

sht40_t *sht40_create(void *ctx, uint8_t dev_addr, sht40_i2c_write_fn i2c_write,
                      sht40_i2c_read_fn i2c_read, sht40_delay_ms_fn delay_ms) {
  if (i2c_write == NULL || i2c_read == NULL || delay_ms == NULL) {
    return NULL;
  }
  if (instance_used) {
    return NULL;
  }
  instance.ctx = ctx;
  instance.dev_addr = dev_addr;
  instance.i2c_write = i2c_write;
  instance.i2c_read = i2c_read;
  instance.delay_ms = delay_ms;
  instance_used = 1;
  return &instance;
}

sht40_status_t sht40_soft_reset(sht40_t *dev) {
  if (dev == NULL) {
    return SHT40_NULL_PTR;
  }
  sht40_status_t status = sht40_send_command(dev, SHT40_CMD_SOFT_RESET);
  if (status != SHT40_OK) {
    return status;
  }
  dev->delay_ms(dev->ctx, SHT40_WAIT_SOFT_RESET_MS);
  return SHT40_OK;
}

sht40_status_t sht40_read_serial(sht40_t *dev, uint32_t *serial) {
  if (dev == NULL || serial == NULL) {
    return SHT40_NULL_PTR;
  }
  sht40_status_t status = sht40_send_command(dev, SHT40_CMD_READ_SERIAL);
  if (status != SHT40_OK) {
    return status;
  }
  dev->delay_ms(dev->ctx, SHT40_WAIT_SERIAL_MS);

  uint16_t word_high, word_low;
  status = sht40_read_words(dev, &word_high, &word_low);
  if (status != SHT40_OK) {
    return status;
  }
  *serial = ((uint32_t)word_high << 16) | word_low;
  return SHT40_OK;
}

sht40_status_t sht40_start_measurement(sht40_t *dev,
                                       sht40_precision_t precision) {
  if (dev == NULL) {
    return SHT40_NULL_PTR;
  }
  return sht40_send_command(dev, (uint8_t)precision);
}

sht40_status_t sht40_start_heater(sht40_t *dev, sht40_heater_t heater) {
  if (dev == NULL) {
    return SHT40_NULL_PTR;
  }
  return sht40_send_command(dev, (uint8_t)heater);
}

uint32_t sht40_measurement_wait_ms(sht40_precision_t precision) {
  switch (precision) {
  case SHT40_PRECISION_LOW:
    return SHT40_WAIT_LOW_MS;
  case SHT40_PRECISION_MEDIUM:
    return SHT40_WAIT_MEDIUM_MS;
  case SHT40_PRECISION_HIGH:
  default:
    return SHT40_WAIT_HIGH_MS;
  }
}

uint32_t sht40_heater_wait_ms(sht40_heater_t heater) {
  switch (heater) {
  case SHT40_HEATER_200MW_100MS:
  case SHT40_HEATER_110MW_100MS:
  case SHT40_HEATER_20MW_100MS:
    return SHT40_WAIT_HEATER_100MS_MS;
  case SHT40_HEATER_200MW_1S:
  case SHT40_HEATER_110MW_1S:
  case SHT40_HEATER_20MW_1S:
  default:
    return SHT40_WAIT_HEATER_1S_MS;
  }
}

sht40_status_t sht40_read_measurement(sht40_t *dev, float *temperature,
                                      float *humidity) {
  if (dev == NULL || temperature == NULL || humidity == NULL) {
    return SHT40_NULL_PTR;
  }
  uint16_t temperature_ticks, humidity_ticks;
  sht40_status_t status =
      sht40_read_words(dev, &temperature_ticks, &humidity_ticks);
  if (status != SHT40_OK) {
    return status;
  }
  *temperature = sht40_ticks_to_celsius(temperature_ticks);
  *humidity = sht40_ticks_to_humidity(humidity_ticks);
  return SHT40_OK;
}

sht40_status_t sht40_measure(sht40_t *dev, sht40_precision_t precision,
                             float *temperature, float *humidity) {
  if (dev == NULL || temperature == NULL || humidity == NULL) {
    return SHT40_NULL_PTR;
  }
  sht40_status_t status = sht40_start_measurement(dev, precision);
  if (status != SHT40_OK) {
    return status;
  }
  dev->delay_ms(dev->ctx, sht40_measurement_wait_ms(precision));
  return sht40_read_measurement(dev, temperature, humidity);
}