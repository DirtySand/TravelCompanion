#include "i2c_bus.h"
#include "board_config.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#define I2C_TIMEOUT_BASE_US 5000u
#define I2C_TIMEOUT_PER_BYTE_US 500u
#define I2C_RECOVERY_CLOCKS 9
#define I2C_RECOVERY_HALF_PERIOD_US 5

static uint32_t timeout_count;

static void attach_pins(void) {
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);
}

static void release_line(uint pin) { gpio_set_dir(pin, GPIO_IN); }

static void pull_line_low(uint pin) {
  gpio_put(pin, 0);
  gpio_set_dir(pin, GPIO_OUT);
}

static void recover(void) {
  gpio_init(I2C_SDA_PIN);
  gpio_init(I2C_SCL_PIN);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);
  for (int i = 0; i < I2C_RECOVERY_CLOCKS && !gpio_get(I2C_SDA_PIN); i++) {
    pull_line_low(I2C_SCL_PIN);
    sleep_us(I2C_RECOVERY_HALF_PERIOD_US);
    release_line(I2C_SCL_PIN);
    sleep_us(I2C_RECOVERY_HALF_PERIOD_US);
  }
  pull_line_low(I2C_SDA_PIN);
  sleep_us(I2C_RECOVERY_HALF_PERIOD_US);
  release_line(I2C_SCL_PIN);
  sleep_us(I2C_RECOVERY_HALF_PERIOD_US);
  release_line(I2C_SDA_PIN);
  sleep_us(I2C_RECOVERY_HALF_PERIOD_US);

  i2c_init(I2C_PORT, I2C_BAUDRATE);
  attach_pins();
}

static uint timeout_us(size_t length) {
  return I2C_TIMEOUT_BASE_US + (uint)length * I2C_TIMEOUT_PER_BYTE_US;
}

static int finish(int result) {
  if (result == PICO_ERROR_TIMEOUT) {
    timeout_count++;
    recover();
  }
  return result;
}

void i2c_bus_init(void) {
  i2c_init(I2C_PORT, I2C_BAUDRATE);
  attach_pins();
}

int i2c_bus_write(i2c_inst_t *i2c, uint8_t addr, const uint8_t *data,
                  size_t length, bool nostop) {
  return finish(i2c_write_timeout_us(i2c, addr, data, length, nostop,
                                     timeout_us(length)));
}

int i2c_bus_read(i2c_inst_t *i2c, uint8_t addr, uint8_t *data, size_t length,
                 bool nostop) {
  return finish(
      i2c_read_timeout_us(i2c, addr, data, length, nostop, timeout_us(length)));
}

uint32_t i2c_bus_timeout_count(void) { return timeout_count; }
