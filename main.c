#include "drivers/bme680/bme680.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
#define I2C_PORT i2c1
#define ADDR_BME680 0x77
#define I2C_BAUDRATE 100000

int bme680_port_i2c_read(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                         uint8_t *data, uint8_t length);

int bme680_port_i2c_write(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                          const uint8_t *data, uint8_t length);

int main() {
  stdio_init_all();
  sleep_ms(1000);
  i2c_init(I2C_PORT, I2C_BAUDRATE);
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);

  bme680_t *bme680 = bme680_create(I2C_PORT, ADDR_BME680, bme680_port_i2c_read,
                                   bme680_port_i2c_write);

  if (bme680 == NULL) {
    printf("Failed to create BME680 instance\n");
    return -1;
  }
  uint8_t chip_id = 0;

  bme680_status_t status = bme680_read_chip_id(bme680, &chip_id);
  if (status != BME680_OK) {
    printf("Failed to read chip ID: %d\n", status);
  } else {
    printf("Chip ID: 0x%02X\n", chip_id);
  }
  sleep_ms(1000);
  
  while (true) {
    tight_loop_contents();
  }
}