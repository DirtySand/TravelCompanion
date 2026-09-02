#include "drivers/SSD1351U3/ssd1351.h"
#include "drivers/bme680/bme680.h"
#include "drivers/ds3231/ds3231.h"
#include "drivers/ds3231/ds3231_config.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "timezone.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
#define I2C_PORT i2c1
#define BME680_I2C_ADDRESS 0x77
#define DS3231_I2C_ADDRESS 0x68
#define I2C_BAUDRATE 100000
#define CENTURY_BASE 2000

int bme680_port_i2c_read(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                         uint8_t *data, uint8_t length);

int bme680_port_i2c_write(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                          const uint8_t *data, uint8_t length);

int ds3231_port_i2c_read(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                         uint8_t *data, uint8_t length);
int ds3231_port_i2c_write(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                          const uint8_t *data, uint8_t length);

int main() {
  stdio_init_all();
  sleep_ms(1000);
  i2c_init(I2C_PORT, I2C_BAUDRATE);
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);

  bme680_t *bme680 = bme680_create(I2C_PORT, BME680_I2C_ADDRESS,
                                   bme680_port_i2c_read, bme680_port_i2c_write);

  if (bme680 == NULL) {
    printf("Failed to create BME680 instance\n");
    return -1;
  }
  uint8_t bme680_chip_id = 0;

  bme680_status_t bme680_status = bme680_read_chip_id(bme680, &bme680_chip_id);
  if (bme680_status != BME680_OK) {
    printf("Failed to read chip ID: %d\n", bme680_status);
  } else {
    printf("Chip ID: 0x%02X\n", bme680_chip_id);
  }

  ssd1351_port_hw_init();
  ssd1351_io_t io = ssd1351_port_get_io();
  ssd1351_t *oled = ssd1351_get_instance();
  ssd1351_init(oled, &io);
  ssd1351_fill(oled, 0x0000);

  ds3231_t *ds3231 = ds3231_create(I2C_PORT, DS3231_I2C_ADDRESS,
                                   ds3231_port_i2c_read, ds3231_port_i2c_write);
  if (ds3231 == NULL) {
    printf("Failed to create DS3231 instance\n");
    return -1;
  }
  ds3231_status_t ds3231_status =
      ds3231_set_time_format(ds3231, DS3231_24_HOUR_FORMAT);
  if (ds3231_status != DS3231_OK) {
    printf("DS3231 set time format failed");
    return -1;
  }

  while (true) {

    float temperature, pressure, humidity;
    printf("Starting forced measurement...\n");
    bme680_status = bme680_read_measurement_forced(bme680, &temperature,
                                                   &pressure, &humidity);
    if (bme680_status != BME680_OK) {
      printf("Failed to read measurement: %d\n", bme680_status);
    } else {
      printf("Temperature: %.2f°C, Pressure: %.2f hPa, Humidity: %.2f %%\n",
             temperature, pressure / 100, humidity);
    }

    uint8_t seconds, minutes, hours, hours_utc, day, month, year;
    uint16_t year_with_century;
    ds3231_datetime_t datetime;
    ds3231_status = ds3231_read_datetime(ds3231, &datetime);
    if (ds3231_status != DS3231_OK) {
      printf("Failed to read datetime: %d\n", ds3231_status);
    } else {
      seconds = datetime.seconds;
      minutes = datetime.minutes;
      hours_utc = datetime.hours;
      day = datetime.day;
      month = datetime.month;
      year = datetime.year;
    }
    year_with_century = CENTURY_BASE + year;
    if (is_dst_active(day, month, year_with_century, hours_utc)) {
      hours = hours_utc + 1;
    } else {
      hours = hours_utc;
    }
    hours %= 24;

    char buffer[64];
    uint16_t fg_color = 0xFFFF;
    uint16_t bg_color = 0x0000;
    snprintf(buffer, sizeof(buffer),
             "Temp: %.1f"
             "\x7f"
             "C",
             temperature);
    ssd1351_draw_string(oled, 0, 0, buffer, fg_color, bg_color);
    snprintf(buffer, sizeof(buffer), "Pres: %.2f hPa", pressure / 100);
    ssd1351_draw_string(oled, 0, 10, buffer, fg_color, bg_color);
    snprintf(buffer, sizeof(buffer), "Hum:  %.2f %%", humidity);
    ssd1351_draw_string(oled, 0, 20, buffer, fg_color, bg_color);
    snprintf(buffer, sizeof(buffer), "Time: %d:%d:%d", hours, minutes, seconds);
    ssd1351_draw_string(oled, 0, 30, buffer, fg_color, bg_color);
    snprintf(buffer, sizeof(buffer), "Date: %d-%d-%d", day, month,
             year_with_century);
    ssd1351_draw_string(oled, 0, 40, buffer, fg_color, bg_color);

    sleep_ms(1000);
  }
}