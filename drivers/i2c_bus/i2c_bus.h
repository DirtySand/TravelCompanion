#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "hardware/i2c.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void i2c_bus_init(void);
int i2c_bus_write(i2c_inst_t *i2c, uint8_t addr, const uint8_t *data,
                  size_t length, bool nostop);
int i2c_bus_read(i2c_inst_t *i2c, uint8_t addr, uint8_t *data, size_t length,
                 bool nostop);
uint32_t i2c_bus_timeout_count(void);

#endif // I2C_BUS_H
