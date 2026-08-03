#include "ssd1351_port.h"
#include "board_config.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

typedef struct {
  spi_inst_t *spi;
  uint pin_cs;
  uint pin_dc;
  uint pin_rst;
} ssd1351_hw_t;

static ssd1351_hw_t oled_hw = {
    .spi = SSD1351_SPI,
    .pin_cs = SSD1351_PIN_CS,
    .pin_dc = SSD1351_PIN_DC,
    .pin_rst = SSD1351_PIN_RST,
};

void ssd1351_port_hw_init(void) {
  spi_init(oled_hw.spi, 10 * 1000 * 1000);
  spi_set_format(oled_hw.spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
  gpio_set_function(SSD1351_PIN_CLK, GPIO_FUNC_SPI);
  gpio_set_function(SSD1351_PIN_DIN, GPIO_FUNC_SPI);

  gpio_init(oled_hw.pin_cs);
  gpio_set_dir(oled_hw.pin_cs, GPIO_OUT);
  gpio_put(oled_hw.pin_cs, 1);
  gpio_init(oled_hw.pin_dc);
  gpio_set_dir(oled_hw.pin_dc, GPIO_OUT);
  gpio_put(oled_hw.pin_dc, 0);
  gpio_init(oled_hw.pin_rst);
  gpio_set_dir(oled_hw.pin_rst, GPIO_OUT);
  gpio_put(oled_hw.pin_rst, 1);
}

static void port_reset(void *ctx) {
  ssd1351_hw_t *hw = (ssd1351_hw_t *)ctx;
  gpio_put(hw->pin_rst, 1);
  sleep_ms(1);
  gpio_put(hw->pin_rst, 0);
  sleep_us(10);
  gpio_put(hw->pin_rst, 1);
  sleep_ms(100);
}

static void port_write_cmd(void *ctx, uint8_t cmd) {
  ssd1351_hw_t *hw = (ssd1351_hw_t *)ctx;
  gpio_put(hw->pin_dc, 0);
  gpio_put(hw->pin_cs, 0);
  spi_write_blocking(hw->spi, &cmd, 1);
  gpio_put(hw->pin_cs, 1);
}

static void port_write_data(void *ctx, const uint8_t *data, size_t len) {
  ssd1351_hw_t *hw = (ssd1351_hw_t *)ctx;
  gpio_put(hw->pin_dc, 1);
  gpio_put(hw->pin_cs, 0);
  spi_write_blocking(hw->spi, data, len);
  gpio_put(hw->pin_cs, 1);
}

static void port_delay_ms(void *ctx, uint32_t ms) {
  (void)ctx;
  sleep_ms(ms);
}

ssd1351_io_t ssd1351_port_get_io(void) {
  return (ssd1351_io_t){
      .write_cmd = port_write_cmd,
      .write_data = port_write_data,
      .reset = port_reset,
      .delay_ms = port_delay_ms,
      .ctx = &oled_hw,
  };
}