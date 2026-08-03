#include "ssd1351.h"
#include "fonts/font_8x8/font_8x8.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

typedef struct {
  uint8_t cmd;
  uint8_t args[4];
  uint8_t nargs;
  uint16_t post_delay_ms;
} init_cmd_t;

struct ssd1351 {
  ssd1351_io_t io;
};

static const init_cmd_t init_seq[] = {
    {0xFD, {0x12}, 1, 0},
    {0xFD, {0xB1}, 1, 0},
    {0xAE, {0}, 0, 0},
    {0xB3, {0xF1}, 1, 0},
    {0xCA, {0x7F}, 1, 0},
    {0xA2, {0x00}, 1, 0},
    {0xA0, {0x74}, 1, 0},
    {0x15, {0x00, 0x7F}, 2, 0},
    {0x75, {0x00, 0x7F}, 2, 0},
    {0xB5, {0x00}, 1, 0},
    {0xAB, {0x01}, 1, 0},
    {0xB1, {0x32}, 1, 0},
    {0xBE, {0x05}, 1, 0},
    {0xB4, {0xA0, 0xB5, 0x55}, 3, 0},
    {0xB6, {0x01}, 1, 0},
    {0xA6, {0}, 0, 0},
    {0xC1, {0xC8, 0x80, 0xC8}, 3, 0},
    {0xC7, {0x0F}, 1, 0},
    {0xAF, {0}, 0, 200},
};

static struct ssd1351 g_instance;

ssd1351_t *ssd1351_get_instance(void) { return &g_instance; }

void ssd1351_reset(ssd1351_t *dev) { dev->io.reset(dev->io.ctx); }

static void write_command(ssd1351_t *dev, uint8_t command) {
  dev->io.write_cmd(dev->io.ctx, command);
}

static void write_data(ssd1351_t *dev, const uint8_t *data, size_t len) {
  dev->io.write_data(dev->io.ctx, data, len);
}

static void delay_ms(ssd1351_t *dev, uint32_t ms) {
  dev->io.delay_ms(dev->io.ctx, ms);
}

static void run_init_sequence(ssd1351_t *dev) {
    for (size_t i = 0; i < ARRAY_LEN(init_seq); i++) {
        const init_cmd_t *c = &init_seq[i];
        write_command(dev, c->cmd);
        if (c->nargs)         write_data(dev, c->args, c->nargs);
        if (c->post_delay_ms) delay_ms(dev, c->post_delay_ms);
    }
}

void ssd1351_init(ssd1351_t *dev, const ssd1351_io_t *io) {
    dev->io = *io;
    ssd1351_reset(dev);
    run_init_sequence(dev);
}

void ssd1351_set_window(ssd1351_t *dev, uint8_t x0, uint8_t y0,
                        uint8_t x1, uint8_t y1) {
    uint8_t col[] = { x0, x1 };
    uint8_t row[] = { y0, y1 };
    write_command(dev, 0x15); write_data(dev, col, 2);
    write_command(dev, 0x75); write_data(dev, row, 2);
    write_command(dev, 0x5C);
}

void ssd1351_draw_pixel(ssd1351_t *dev, uint8_t x, uint8_t y, uint16_t color) {
    if (x > 127 || y > 127) return;
    ssd1351_set_window(dev, x, y, x, y);
    uint8_t px[] = { color >> 8, color & 0xFF };
    write_data(dev, px, 2);
}

void ssd1351_fill_rect(ssd1351_t *dev, uint8_t x, uint8_t y,
                       uint8_t w, uint8_t h, uint16_t color) {
    if (x >= 128 || y >= 128) return;
    if (x + w > 128) w = 128 - x;
    if (y + h > 128) h = 128 - y;

    ssd1351_set_window(dev, x, y, x + w - 1, y + h - 1);

    uint8_t line[128 * 2];
    uint8_t hi = color >> 8, lo = color & 0xFF;
    for (uint8_t i = 0; i < w; i++) { line[i*2] = hi; line[i*2+1] = lo; }
    for (uint8_t r = 0; r < h; r++) write_data(dev, line, w * 2);
}

void ssd1351_fill(ssd1351_t *dev, uint16_t color) {
    ssd1351_fill_rect(dev, 0, 0, 128, 128, color);
}

void ssd1351_draw_char(ssd1351_t *dev, uint8_t x, uint8_t y,
                       char c, uint16_t fg, uint16_t bg) {
    if (x > 120 || y > 120) return;
    const uint8_t *glyph = font8x8_glyph(c);
    uint8_t cell[8 * 8 * 2];

    size_t p = 0;
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            uint16_t color = ((glyph[row] >> col) & 1) ? fg : bg;
            cell[p++] = color >> 8;
            cell[p++] = color & 0xFF;
        }
    }

    ssd1351_set_window(dev, x, y, x + 7, y + 7);
    write_data(dev, cell, sizeof(cell));
}

void ssd1351_draw_string(ssd1351_t *dev, uint8_t x, uint8_t y,
                         const char *s, uint16_t fg, uint16_t bg) {
    while (*s) {
        ssd1351_draw_char(dev, x, y, *s, fg, bg);
        x += 8;
        if (x > 120) break;
        s++;
    }
}