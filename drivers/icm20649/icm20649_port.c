#include "icm20649_port.h"
#include "hardware/i2c.h"
#include "pico/time.h"

#define ICM20649_PORT_MAX_WRITE 8

int icm20649_port_i2c_read(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                           uint8_t *data, uint8_t length) {
  i2c_inst_t *i2c = (i2c_inst_t *)ctx;
  int result = i2c_write_blocking(i2c, dev_addr, &reg_addr, 1, true);
  if (result < 0) {
    return result;
  }
  return i2c_read_blocking(i2c, dev_addr, data, length, false);
}

int icm20649_port_i2c_write(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                            const uint8_t *data, uint8_t length) {
  i2c_inst_t *i2c = (i2c_inst_t *)ctx;
  uint8_t buffer[ICM20649_PORT_MAX_WRITE + 1];
  if (length > ICM20649_PORT_MAX_WRITE) {
    return -1;
  }
  buffer[0] = reg_addr;
  for (uint8_t i = 0; i < length; i++) {
    buffer[i + 1] = data[i];
  }
  return i2c_write_blocking(i2c, dev_addr, buffer, (size_t)length + 1, false);
}

void icm20649_port_delay_ms(void *ctx, uint32_t ms) {
  (void)ctx;
  sleep_ms(ms);
}
