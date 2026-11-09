#ifndef QMC5883L_PORT_H
#define QMC5883L_PORT_H

#include <stdint.h>

int qmc5883l_port_i2c_read(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                           uint8_t *data, uint8_t length);
int qmc5883l_port_i2c_write(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                            const uint8_t *data, uint8_t length);
void qmc5883l_port_delay_ms(void *ctx, uint32_t ms);

#endif // QMC5883L_PORT_H
