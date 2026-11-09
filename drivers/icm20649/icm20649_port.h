#ifndef ICM20649_PORT_H
#define ICM20649_PORT_H

#include <stdint.h>

int icm20649_port_i2c_read(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                           uint8_t *data, uint8_t length);
int icm20649_port_i2c_write(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                            const uint8_t *data, uint8_t length);
void icm20649_port_delay_ms(void *ctx, uint32_t ms);

#endif // ICM20649_PORT_H
