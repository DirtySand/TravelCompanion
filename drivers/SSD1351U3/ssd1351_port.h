#ifndef SSD1351_PORT_H
#define SSD1351_PORT_H

#include <stddef.h>
#include <stdint.h>
typedef struct {
  void (*write_cmd)(void *ctx, uint8_t cmd);
  void (*write_data)(void *ctx, const uint8_t *data, size_t len);
  void (*reset)(void *ctx);
  void (*delay_ms)(void *ctx, uint32_t ms);
  void *ctx;
} ssd1351_io_t;

void ssd1351_port_hw_init(void);
ssd1351_io_t ssd1351_port_get_io(void);

#endif