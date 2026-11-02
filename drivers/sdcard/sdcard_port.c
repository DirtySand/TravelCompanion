#include "sdcard_port.h"
#include "board_config.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#define SD_IDLE_BYTE 0xFFu

void sdcard_port_hw_init(void) {
  gpio_set_function(SD_PIN_MISO, GPIO_FUNC_SPI);
  gpio_pull_up(SD_PIN_MISO);
  gpio_init(SD_PIN_CS);
  gpio_set_dir(SD_PIN_CS, GPIO_OUT);
  gpio_put(SD_PIN_CS, 1);
}

static void port_transfer(void *ctx, const uint8_t *tx, uint8_t *rx,
                          size_t length) {
  (void)ctx;
  if (tx != NULL && rx != NULL) {
    spi_write_read_blocking(SD_SPI, tx, rx, length);
  } else if (tx != NULL) {
    spi_write_blocking(SD_SPI, tx, length);
  } else if (rx != NULL) {
    spi_read_blocking(SD_SPI, SD_IDLE_BYTE, rx, length);
  } else {
    uint8_t discard;
    for (size_t i = 0; i < length; i++) {
      spi_read_blocking(SD_SPI, SD_IDLE_BYTE, &discard, 1);
    }
  }
}

static void port_select(void *ctx, bool selected) {
  (void)ctx;
  gpio_put(SD_PIN_CS, selected ? 0 : 1);
}

static void port_set_clock(void *ctx, bool fast) {
  (void)ctx;
  // The fast rate equals the OLED rate, so the shared bus never has to be
  // switched back after an SD transfer.
  spi_set_baudrate(SD_SPI, fast ? SPI0_BAUDRATE : SD_SPI_INIT_BAUDRATE);
}

static uint32_t port_millis(void *ctx) {
  (void)ctx;
  return to_ms_since_boot(get_absolute_time());
}

sdcard_io_t sdcard_port_get_io(void) {
  return (sdcard_io_t){
      .transfer = port_transfer,
      .select = port_select,
      .set_clock = port_set_clock,
      .millis = port_millis,
      .ctx = NULL,
  };
}
