#ifndef SSD1351_H
#define SSD1351_H

#include "ssd1351_port.h"
#include <stdint.h>

typedef struct ssd1351 ssd1351_t;
ssd1351_t *ssd1351_get_instance(void);
void ssd1351_init(ssd1351_t *dev, const ssd1351_io_t *io);
void ssd1351_reset(ssd1351_t *dev);
void ssd1351_fill(ssd1351_t *dev, uint16_t color);
void ssd1351_set_window(ssd1351_t *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ssd1351_draw_pixel(ssd1351_t *dev, uint8_t x, uint8_t y, uint16_t color);
void ssd1351_fill_rect(ssd1351_t *dev, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
void ssd1351_draw_char(ssd1351_t *dev, uint8_t x, uint8_t y, char c, uint16_t fg, uint16_t bg);
void ssd1351_draw_string(ssd1351_t *dev, uint8_t x, uint8_t y, const char *s, uint16_t fg, uint16_t bg);

#endif