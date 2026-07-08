#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
#define I2C_PORT i2c1
#define ADDR_BME680 0x77

int main() {
  stdio_init_all();
  sleep_ms(2000);

  i2c_init(I2C_PORT, 100 * 1000);
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);

  while (true) {

    printf("Wysylanie danych do BME680\n");
    uint8_t bme_reg[1] = {0xD0};
    uint8_t bme_id[1] = {0x00};
    int result = i2c_write_blocking(I2C_PORT, ADDR_BME680, &bme_reg[0], 1, true);
    if (result < 0) {
      printf("Blad podczas wysylania danych do BME680\n");
    }
    result = i2c_read_blocking(I2C_PORT, ADDR_BME680, &bme_id[0], 1, false);
    if (result < 0) {
      printf("Blad podczas odczytu danych z BME680\n");
    } else {
      printf("Odczytano dane z BME680: 0x%02X\n", bme_id[0]);
    }
    sleep_ms(50000);
  }
}