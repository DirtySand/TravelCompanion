#include "hardware/i2c.h"

int ds3231_port_i2c_read(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                         uint8_t *data, uint8_t length) {
  i2c_inst_t *i2c = (i2c_inst_t *)ctx;

  int result = i2c_write_blocking(i2c, dev_addr, &reg_addr, 1, true);
  if (result < 0) {
    return result;
  }
  return i2c_read_blocking(i2c, dev_addr, data, length, false);
}

int ds3231_port_i2c_write(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                          const uint8_t *data, uint8_t length) {
  i2c_inst_t *i2c = (i2c_inst_t *)ctx;

  uint8_t buffer[16];
  if (length + 1 > sizeof(buffer)) {
    return -1;
  }
  buffer[0] = reg_addr;
  for (int i = 0; i < length; i++) {
    buffer[i + 1] = data[i];
  }
  return i2c_write_blocking(i2c, dev_addr, buffer, length + 1, false);
}