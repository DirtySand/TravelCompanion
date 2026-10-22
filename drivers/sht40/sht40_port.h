#ifndef SHT40_PORT_H
#define SHT40_PORT_H

#include <stddef.h>
#include <stdint.h>

int sht40_port_i2c_write(void *ctx, uint8_t dev_addr, const uint8_t *data,
                         size_t length);
int sht40_port_i2c_read(void *ctx, uint8_t dev_addr, uint8_t *data,
                        size_t length);
void sht40_port_delay_ms(void *ctx, uint32_t ms);

#endif // SHT40_PORT_H