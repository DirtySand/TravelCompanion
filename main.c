#include "board_config.h"
#include "datetime_utils.h"
#include "drivers/SSD1351U3/ssd1351.h"
#include "drivers/bme680/bme680.h"
#include "drivers/ds3231/ds3231.h"
#include "drivers/ds3231/ds3231_config.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "power.h"
#include "timezone.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

int bme680_port_i2c_read(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                         uint8_t *data, uint8_t length);

int bme680_port_i2c_write(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                          const uint8_t *data, uint8_t length);

int ds3231_port_i2c_read(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                         uint8_t *data, uint8_t length);
int ds3231_port_i2c_write(void *ctx, uint8_t dev_addr, uint8_t reg_addr,
                          const uint8_t *data, uint8_t length);

typedef enum {
  MODE_MANUAL,
  MODE_BACKPACK,
} app_mode_t;

static volatile bool button_event = false;

static void button_isr(uint gpio, uint32_t events) {
  static uint64_t last_interrupt_time = 0;
  uint64_t current_time = time_us_64();
  if (current_time - last_interrupt_time < BUTTON_DEBOUNCE_US) {
    return;
  }
  last_interrupt_time = current_time;
  if (gpio == BUTTON_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
    button_event = true;
  }
}

static void display_datetime_local(ssd1351_t *oled,
                                   const ds3231_datetime_t *datetime) {
  uint16_t full_year = CENTURY_BASE + datetime->year;
  char buffer[32];
  uint16_t fg_color = 0xFFFF;
  uint16_t bg_color = 0x0000;
  datetime_to_local((ds3231_datetime_t *)datetime, (ds3231_datetime_t *)datetime, CENTURY_BASE);
  snprintf(buffer, sizeof(buffer), "Time: %02d:%02d:%02d", datetime->hours,
           datetime->minutes, datetime->seconds);
  ssd1351_draw_string(oled, 0, 30, buffer, fg_color, bg_color);
  snprintf(buffer, sizeof(buffer), "Date: %02d-%02d-%04d", datetime->day,
           datetime->month, full_year);
  ssd1351_draw_string(oled, 0, 40, buffer, fg_color, bg_color);
}

static void display_backpack_clock(ssd1351_t *oled,
                                   const ds3231_datetime_t *datetime) {
  char buffer[32];
  uint16_t full_year = CENTURY_BASE + datetime->year;
  datetime_to_local((ds3231_datetime_t *)datetime, (ds3231_datetime_t *)datetime, CENTURY_BASE);
  snprintf(buffer, sizeof(buffer), "Time: %02d:%02d", datetime->hours, datetime->minutes);
  ssd1351_draw_string(oled, 0, 30, buffer, 0x3000, 0x0000);
  snprintf(buffer, sizeof(buffer), "Date: %02d-%02d-%04d", datetime->day,
           datetime->month, CENTURY_BASE + datetime->year);
  ssd1351_draw_string(oled, 0, 40, buffer, 0x3000, 0x0000);

}

static void display_sensor_data(ssd1351_t *oled, float temperature,
                                float pressure, float humidity) {
  char buffer[32];
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
}

static ds3231_status_t arm_next_alarm(ds3231_t *dev) {
  ds3231_datetime_t datetime;
  ds3231_status_t status = ds3231_read_datetime(dev, &datetime);
  if (status != DS3231_OK) {
    return status;
  }
  datetime_add_seconds(&datetime, BACKPACK_INTERVAL_SECONDS, CENTURY_BASE);
  return ds3231_set_alarm_1(dev, datetime.hours, datetime.minutes,
                            datetime.seconds, datetime.day,
                            DS3231_ALARM1_MATCH_DATE);
}

static void take_sample(bme680_t *bme680, ds3231_t *ds3231) {
  float temperature, pressure, humidity;
  ds3231_datetime_t datetime;
  ds3231_read_datetime(ds3231, &datetime);
  if (bme680_read_measurement_forced(bme680, &temperature, &pressure,
                                     &humidity) == BME680_OK) {
    printf("Sample taken at %02d:%02d:%02d on %02d-%02d-%04d: Temp=%.2fC, "
           "Pres=%.2fhPa, Hum=%.2f%%\n",
           datetime.hours, datetime.minutes, datetime.seconds, datetime.day,
           datetime.month, CENTURY_BASE + datetime.year, temperature,
           pressure / 100, humidity);
  } else {
    printf("Failed to take sample at %02d:%02d:%02d on %02d-%02d-%04d\n",
           datetime.hours, datetime.minutes, datetime.seconds, datetime.day,
           datetime.month, CENTURY_BASE + datetime.year);
  }
}

int main() {
  set_sys_clock_khz(SYS_CLOCK_KHZ, true);
  stdio_init_all();
  sleep_ms(1000);
  i2c_init(I2C_PORT, I2C_BAUDRATE);
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);

  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, GPIO_IN);
  gpio_pull_up(BUTTON_PIN);
  gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true,
                                     &button_isr);

  gpio_init(RTC_INT_PIN);
  gpio_set_dir(RTC_INT_PIN, GPIO_IN);
  gpio_pull_up(RTC_INT_PIN);

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
  ssd1351_set_rotation(oled, SSD1351_ROT_90);
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
    printf("DS3231 set time format failed\n");
    return -1;
  }

  bool time_invalid = false;
  ds3231_status = ds3231_check_oscillator_stopped(ds3231, &time_invalid);
  if (ds3231_status != DS3231_OK || time_invalid) {
    time_invalid = true;
    printf("DS3231 time invalid\n");
    ssd1351_draw_string(oled, 0, 0, "RTC time invalid", 0xF800, 0x0000);
  }

  ds3231_enable_alarm1_interrupt(ds3231, true);
  ds3231_clear_alarm1_flag(ds3231);
  app_mode_t mode = MODE_MANUAL;

  while (true) {
    if (mode == MODE_MANUAL) {
      float temperature, pressure, humidity;
      bme680_status = bme680_read_measurement_forced(bme680, &temperature,
                                                     &pressure, &humidity);
      if (bme680_status != BME680_OK) {
        printf("Failed to read measurement: %d\n", bme680_status);
      } else {
        printf("Temperature: %.2f°C, Pressure: %.2f hPa, Humidity: %.2f %%\n",
               temperature, pressure / 100, humidity);
        display_sensor_data(oled, temperature, pressure, humidity);
      }

      ds3231_datetime_t datetime;
      if (ds3231_read_datetime(ds3231, &datetime) != DS3231_OK) {
        printf("Failed to read datetime: %d\n", ds3231_status);
      } else {
        display_datetime_local(oled, &datetime);
      }

      if (button_event) {
        button_event = false;
        if (time_invalid) {
          ssd1351_draw_string(oled, 0, 0, "RTC time invalid", 0xF800, 0x0000);
        } else {
          mode = MODE_BACKPACK;
          ssd1351_fill(oled, 0x0000);
          ssd1351_draw_string(oled, 0, 0, "Backpack mode", 0x2000, 0x0000);
          ds3231_datetime_t current_time;
          if (ds3231_read_datetime(ds3231, &current_time) == DS3231_OK) {
            display_backpack_clock(oled, &current_time);
          }
        }
      }
      sleep_ms(1000);
    } else {
      ds3231_clear_alarm1_flag(ds3231);
      if (arm_next_alarm(ds3231) != DS3231_OK) {
        mode = MODE_MANUAL;
        continue;
      }
      button_event = false;
      power_dormant_until_pins(RTC_INT_PIN, BUTTON_PIN);
      bool alarm_triggered = false;
      ds3231_is_alarm1_triggered(ds3231, &alarm_triggered);
      if (alarm_triggered) {
        take_sample(bme680, ds3231);
        ds3231_datetime_t current_time;
        if (ds3231_read_datetime(ds3231, &current_time) == DS3231_OK) {
          display_backpack_clock(oled, &current_time);
        }
      }
      if (button_event || gpio_get(BUTTON_PIN) == 0) {
        button_event = false;
        ds3231_clear_alarm1_flag(ds3231);
        mode = MODE_MANUAL;
        ssd1351_fill(oled, 0x0000);
      }
    }
  }
}