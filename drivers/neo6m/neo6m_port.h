#ifndef NEO6M_PORT_H
#define NEO6M_PORT_H

#include <stddef.h>
#include <stdint.h>

void neo6m_port_hw_init(void);
int neo6m_port_uart_read(void *ctx, uint8_t *data, size_t max_length);
int neo6m_port_uart_write(void *ctx, const uint8_t *data, size_t length);
uint32_t neo6m_port_dropped_bytes(void);

#endif // NEO6M_PORT_H