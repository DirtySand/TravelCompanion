#include "app.h"
#include "bme680_port.h"
#include "board_config.h"
#include "ds3231_port.h"
#include "hardware/clocks.h"
#include "i2c_bus.h"
#include "neo6m_port.h"
#include "pico/rand.h"
#include "pico/stdio_usb.h"
#include "power.h"
#include "sd_log.h"
#include "sht40_port.h"
#include "ssd1351_port.h"
#include <stdio.h>

#define GPS_AUTO_SYNC_DEFAULT false
#define USB_LOG_WAIT_MS 10000
#define USB_LOG_POST_CONNECT_MS 50
#define SELF_TEST_SHOW_MS 5000

typedef enum {
  FAULT_BME680 = 1u << 0,
  FAULT_SHT40 = 1u << 1,
  FAULT_RTC = 1u << 2,
  FAULT_MAG = 1u << 3,
  FAULT_IMU = 1u << 4,
} fault_t;

static app_t app;

static void wait_for_usb_log(ssd1351_t *oled) {
  if (gpio_get(buttons_pin(BUTTON_BACK)) != 0) {
    return;
  }
  ui_draw_centered(oled, 2, "USB log", UI_COLOR_WHITE);
  ui_draw_centered(oled, 4, "Open terminal", UI_COLOR_GRAY);
  ui_draw_centered(oled, 8, "Any button:skip", UI_COLOR_GRAY);
  buttons_wait_release(BUTTON_BACK);
  absolute_time_t deadline = make_timeout_time_ms(USB_LOG_WAIT_MS);
  int32_t shown = -1;
  while (!stdio_usb_connected() && !buttons_has_events() &&
         !time_reached(deadline)) {
    int32_t left_s =
        (int32_t)((absolute_time_diff_us(get_absolute_time(), deadline) +
                   999999) /
                  1000000);
    if (left_s != shown) {
      char text[16];
      snprintf(text, sizeof(text), "%ld s", (long)left_s);
      ui_draw_centered(oled, 6, text, UI_COLOR_YELLOW);
      shown = left_s;
    }
    sleep_ms(10);
  }
  if (stdio_usb_connected()) {
    sleep_ms(USB_LOG_POST_CONNECT_MS);
    printf("USB log connected\n");
  }
  buttons_take_events();
  ui_clear(oled);
}

static void halt_with_message(ssd1351_t *oled, const char *message) {
  printf("Fatal: %s\n", message);
  ui_clear(oled);
  ui_draw_centered(oled, 4, "Fatal error", UI_COLOR_RED);
  ui_draw_centered(oled, 6, message, UI_COLOR_WHITE);
  while (true) {
    tight_loop_contents();
  }
}

static void show_self_test(ssd1351_t *oled, uint32_t faults) {
  static const struct {
    fault_t fault;
    const char *text;
  } rows[] = {
      {FAULT_BME680, "BME680: FAIL"}, {FAULT_SHT40, "SHT40: FAIL"},
      {FAULT_RTC, "DS3231: FAIL"},    {FAULT_MAG, "QMC5883L: FAIL"},
      {FAULT_IMU, "ICM20649: FAIL"},
  };
  if (faults == 0) {
    return;
  }
  ui_clear(oled);
  ui_draw_centered(oled, 0, "Self-test", UI_COLOR_YELLOW);
  uint8_t row = 2;
  for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
    if (faults & rows[i].fault) {
      ui_draw_line(oled, row++, rows[i].text, UI_COLOR_RED);
    }
  }
  ui_draw_centered(oled, 9, "Any button: go", UI_COLOR_GRAY);
  buttons_take_events();
  absolute_time_t deadline = make_timeout_time_ms(SELF_TEST_SHOW_MS);
  while (!buttons_has_events() && !time_reached(deadline)) {
    sleep_ms(10);
  }
  buttons_take_events();
  ui_clear(oled);
}

static void init_rtc_int_pin(void) {
  gpio_init(RTC_INT_PIN);
  gpio_set_dir(RTC_INT_PIN, GPIO_IN);
  gpio_pull_up(RTC_INT_PIN);
  gpio_set_irq_enabled(RTC_INT_PIN, GPIO_IRQ_EDGE_FALL, true);
}

static ssd1351_t *init_oled(void) {
  ssd1351_port_hw_init();
  ssd1351_io_t io = ssd1351_port_get_io();
  ssd1351_t *oled = ssd1351_get_instance();
  ssd1351_init(oled, &io);
  ssd1351_set_rotation(oled, SSD1351_ROT_0);
  ssd1351_set_contrast(oled, OLED_CONTRAST_MANUAL);
  ssd1351_fill(oled, 0x0000);
  return oled;
}

static bme680_t *init_bme680(void) {
  bme680_t *bme680 =
      bme680_create(I2C_PORT, BME680_I2C_ADDRESS, bme680_port_i2c_read,
                    bme680_port_i2c_write, bme680_port_delay_ms);
  if (bme680 == NULL) {
    printf("BME680 not responding\n");
    return NULL;
  }
  uint8_t chip_id = 0;
  bme680_status_t status = bme680_read_chip_id(bme680, &chip_id);
  if (status != BME680_OK) {
    printf("Failed to read chip ID: %d\n", status);
  } else {
    printf("Chip ID: 0x%02X\n", chip_id);
  }
  return bme680;
}

static sht40_t *init_sht40(bool *responding) {
  sht40_t *sht40 =
      sht40_create(I2C_PORT, SHT40_I2C_ADDRESS, sht40_port_i2c_write,
                   sht40_port_i2c_read, sht40_port_delay_ms);
  if (sht40 == NULL) {
    return NULL;
  }
  sht40_status_t status = sht40_soft_reset(sht40);
  if (status != SHT40_OK) {
    printf("SHT40 soft reset failed: %d\n", status);
  }
  uint32_t serial = 0;
  status = sht40_read_serial(sht40, &serial);
  if (status != SHT40_OK) {
    printf("Failed to read SHT40 serial: %d\n", status);
  } else {
    printf("SHT40 serial: 0x%08lX\n", (unsigned long)serial);
  }
  *responding = (status == SHT40_OK);
  return sht40;
}

static neo6m_t *init_gps(void) {
  neo6m_port_hw_init();
  neo6m_t *gps =
      neo6m_create(NULL, neo6m_port_uart_read, neo6m_port_uart_write);
  if (gps == NULL) {
    printf("Failed to create NEO-6M instance\n");
    return NULL;
  }
  if (neo6m_power_on(gps) != NEO6M_OK) {
    printf("Failed to wake NEO-6M\n");
  }
  return gps;
}

static ds3231_t *init_rtc(bool *time_invalid, bool *responding) {
  ds3231_t *ds3231 = ds3231_create(I2C_PORT, DS3231_I2C_ADDRESS,
                                   ds3231_port_i2c_read, ds3231_port_i2c_write);
  if (ds3231 == NULL) {
    return NULL;
  }
  *time_invalid = true;
  *responding =
      ds3231_set_time_format(ds3231, DS3231_24_HOUR_FORMAT) == DS3231_OK;
  if (!*responding) {
    printf("DS3231 not responding\n");
    return ds3231;
  }
  bool stopped = true;
  ds3231_status_t status = ds3231_check_oscillator_stopped(ds3231, &stopped);
  *time_invalid = (status != DS3231_OK) || stopped;
  if (*time_invalid) {
    printf("DS3231 time invalid\n");
  }
  ds3231_enable_alarm1_interrupt(ds3231, true);
  ds3231_clear_alarm1_flag(ds3231);
  return ds3231;
}

static void init_app_state(app_t *app, bool time_invalid) {
  app->mode = MODE_MANUAL;
  app->card = CARD_ENVIRONMENT;
  app->time_invalid = time_invalid;
  app->clear_screen = true;
  app->gps_enabled = true;
  app->gps_toggled_at = get_absolute_time();
  app->auto_sync = GPS_AUTO_SYNC_DEFAULT;
  app->shift_at = make_timeout_time_ms(BURN_IN_SHIFT_MS);
  app->mag_cal = (compass_cal_t){
      .valid = MAG_CAL_VALID != 0,
      .axis_mask = MAG_CAL_AXIS_MASK,
      .offset = {MAG_CAL_OFFSET_X, MAG_CAL_OFFSET_Y, MAG_CAL_OFFSET_Z},
      .scale = {MAG_CAL_SCALE_X, MAG_CAL_SCALE_Y, MAG_CAL_SCALE_Z},
  };
  attitude_init(&app->attitude, NULL);
  steps_config_t steps_config;
  steps_default_config(&steps_config);
  steps_config.weinberg_k = STEP_WEINBERG_K;
  steps_init(&app->steps, &steps_config);
  trip_reset(&app->trip);
  pdr_reset(&app->pdr);
  fusion_init(&app->fusion, NULL);
  alt_init(&app->alt, NULL);
  app->session = get_rand_32();
  waypoints_start(app);
  app->mag_ok = mag_init(app);
  app->imu_ok = imu_init(app);
  gps_request(app, GPS_CMD_CONFIGURE);
  if (!schedule_from_now(app)) {
    show_error(app, "RTC alarm error");
  }
}

int main() {
  set_sys_clock_khz(SYS_CLOCK_KHZ, true);
  stdio_init_all();
  i2c_bus_init();
  buttons_init(BUTTON_MASK(BUTTON_BACK) | BUTTON_MASK(BUTTON_OK));
  init_rtc_int_pin();
  ssd1351_t *oled = init_oled();
  wait_for_usb_log(oled);

  uint32_t faults = 0;
  bme680_t *bme680 = init_bme680();
  if (bme680 == NULL) {
    faults |= FAULT_BME680;
  }
  bool sht40_responding = false;
  sht40_t *sht40 = init_sht40(&sht40_responding);
  if (sht40 == NULL) {
    halt_with_message(oled, "SHT40 driver");
  }
  if (!sht40_responding) {
    faults |= FAULT_SHT40;
  }
  sd_log_init();
  neo6m_t *gps = init_gps();
  if (gps == NULL) {
    halt_with_message(oled, "GPS driver");
  }
  bool time_invalid = true;
  bool rtc_responding = false;
  ds3231_t *ds3231 = init_rtc(&time_invalid, &rtc_responding);
  if (ds3231 == NULL) {
    halt_with_message(oled, "RTC driver");
  }
  if (!rtc_responding) {
    faults |= FAULT_RTC;
  }

  app.oled = oled;
  app.bme680 = bme680;
  app.sht40 = sht40;
  app.ds3231 = ds3231;
  app.gps = gps;
  init_app_state(&app, time_invalid);
  if (!app.mag_ok) {
    faults |= FAULT_MAG;
  }
  if (!app.imu_ok) {
    faults |= FAULT_IMU;
  }
  show_self_test(oled, faults);
  if (power_watchdog_caused_reboot()) {
    show_error(&app, "Watchdog reset");
  }
  power_watchdog_start();

  absolute_time_t next_refresh = get_absolute_time();
  while (true) {
    if (app.mode == MODE_MANUAL) {
      run_manual_cycle(&app, &next_refresh);
    } else {
      run_backpack_cycle(&app);
    }
  }
}
