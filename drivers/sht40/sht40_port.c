#include "sht40_port.h"
#include "hardware/i2c.h"
#include "pico/time.h"

int sht40_port_i2c_write(void *ctx, uint8_t dev_addr, const uint8_t *data,
                         size_t length) {
  i2c_inst_t *i2c = (i2c_inst_t *)ctx;
  return i2c_write_blocking(i2c, dev_addr, data, length, false);
}

int sht40_port_i2c_read(void *ctx, uint8_t dev_addr, uint8_t *data,
                        size_t length) {
  i2c_inst_t *i2c = (i2c_inst_t *)ctx;
  return i2c_read_blocking(i2c, dev_addr, data, length, false);
}

void sht40_port_delay_ms(void *ctx, uint32_t ms) {
  (void)ctx;
  sleep_ms(ms);
}