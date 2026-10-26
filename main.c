#include "board_config.h"
#include "buttons.h"
#include "datetime_utils.h"
#include "drivers/SSD1351U3/ssd1351.h"
#include "drivers/bme680/bme680.h"
#include "drivers/ds3231/ds3231.h"
#include "drivers/ds3231/ds3231_config.h"
#include "drivers/neo6m/neo6m.h"
#include "drivers/neo6m/neo6m_port.h"
#include "drivers/sht40/sht40.h"
#include "drivers/sht40/sht40_port.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "power.h"
#include "time_sync.h"
#include "timezone.h"
#include "ui.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

// #define WAKE_DEBUG

#define MANUAL_REFRESH_MS 1000
#define GPS_STALE_MS 3000
#define STATUS_MESSAGE_MS 4000
#define GPS_SYNC_TIMEOUT_MS 2500

#define BUTTON_PREV_CARD BUTTON_BOTTOM_LEFT
#define BUTTON_NEXT_CARD BUTTON_BOTTOM_RIGHT
#define BUTTON_OK BUTTON_TOP_RIGHT
#define BUTTON_BACK BUTTON_TOP_LEFT

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

typedef enum {
  CARD_ENVIRONMENT,
  CARD_GPS,
  CARD_BACKPACK,
  CARD_COUNT,
} card_t;

static const char *const card_titles[CARD_COUNT] = {
    [CARD_ENVIRONMENT] = "ENV",
    [CARD_GPS] = "GPS",
    [CARD_BACKPACK] = "PACK",
};

typedef struct {
  char text[UI_COLUMNS + 1];
  uint16_t color;
  absolute_time_t expires;
} status_message_t;

typedef struct {
  ssd1351_t *oled;
  bme680_t *bme680;
  sht40_t *sht40;
  ds3231_t *ds3231;
  neo6m_t *gps;
  app_mode_t mode;
  card_t card;
  bool time_invalid;
  bool clear_screen;
  bool refresh_now;
  bool gps_received;
  absolute_time_t gps_last_rmc;
  bool gps_has_fix;
  neo6m_data_t gps_fix;
  bool gps_fix_utc_valid;
  uint32_t gps_fix_utc_s;
  bool sync_pending;
  absolute_time_t sync_deadline;
  status_message_t status;
} app_t;

static app_t app;

static void set_status(app_t *app, const char *text, uint16_t color) {
  snprintf(app->status.text, sizeof(app->status.text), "%s", text);
  app->status.color = color;
  app->status.expires = make_timeout_time_ms(STATUS_MESSAGE_MS);
  app->refresh_now = true;
}

static void show_error(app_t *app, const char *message) {
  printf("%s\n", message);
  set_status(app, message, UI_COLOR_RED);
}

static bool status_active(const app_t *app) {
  return app->status.text[0] != '\0' && !time_reached(app->status.expires);
}

static void display_backpack_clock(ssd1351_t *oled,
                                   const ds3231_datetime_t *utc) {
  ds3231_datetime_t local;
  char buffer[32];
  datetime_to_local(utc, &local, CENTURY_BASE);
  snprintf(buffer, sizeof(buffer), "Time: %02d:%02d", local.hours,
           local.minutes);
  ssd1351_draw_string(oled, 0, 30, buffer, 0x3000, 0x0000);
  snprintf(buffer, sizeof(buffer), "Date: %02d-%02d-%04d", local.day,
           local.month, CENTURY_BASE + local.year);
  ssd1351_draw_string(oled, 0, 40, buffer, 0x3000, 0x0000);
}

static void display_backpack_screen(ssd1351_t *oled, ds3231_t *ds3231) {
  ssd1351_fill(oled, 0x0000);
  ssd1351_draw_string(oled, 0, 0, "Backpack mode", 0x2000, 0x0000);
  ds3231_datetime_t current_time;
  if (ds3231_read_datetime(ds3231, &current_time) == DS3231_OK) {
    display_backpack_clock(oled, &current_time);
  }
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

static bool restart_backpack_alarm(ds3231_t *ds3231) {
  return ds3231_clear_alarm1_flag(ds3231) == DS3231_OK &&
         arm_next_alarm(ds3231) == DS3231_OK;
}

static bool read_environment(sht40_t *sht40, bme680_t *bme680,
                             float *temperature, float *humidity,
                             float *pressure) {
  bool ok = true;

  sht40_status_t sht40_status =
      sht40_measure(sht40, SHT40_PRECISION_HIGH, temperature, humidity);
  if (sht40_status != SHT40_OK) {
    printf("SHT40 measurement failed: %d\n", sht40_status);
    ok = false;
  }

  float bme680_temperature, bme680_humidity;
  bme680_status_t bme680_status = bme680_read_measurement_forced(
      bme680, &bme680_temperature, pressure, &bme680_humidity);
  if (bme680_status != BME680_OK) {
    printf("BME680 measurement failed: %d\n", bme680_status);
    ok = false;
  }

  return ok;
}

static void take_sample(sht40_t *sht40, bme680_t *bme680, ds3231_t *ds3231) {
  float temperature, pressure, humidity;
  ds3231_datetime_t datetime;
  ds3231_read_datetime(ds3231, &datetime);
  if (read_environment(sht40, bme680, &temperature, &humidity, &pressure)) {
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

#ifdef WAKE_DEBUG
static void display_wake_debug(ssd1351_t *oled, uint32_t wake) {
  static uint32_t wake_count = 0;
  char buffer[32];
  wake_count++;
  snprintf(buffer, sizeof(buffer), "W%lu R%d B%d", (unsigned long)wake_count,
           (wake & POWER_WAKE_RTC) != 0, (wake & POWER_WAKE_BUTTON) != 0);
  ssd1351_draw_string(oled, 0, 60, buffer, 0x3000, 0x0000);
}
#endif

static bool gps_is_fresh(const app_t *app) {
  return app->gps_received &&
         absolute_time_diff_us(app->gps_last_rmc, get_absolute_time()) <
             (int64_t)GPS_STALE_MS * 1000;
}

static bool gps_utc_seconds(const neo6m_data_t *data, uint32_t *seconds) {
  if (!data->time_valid || !data->date_valid || data->seconds > 59) {
    return false;
  }
  ds3231_datetime_t utc = {
      .seconds = data->seconds,
      .minutes = data->minutes,
      .hours = data->hours,
      .day = data->day,
      .month = data->month,
      .year = data->year,
  };
  *seconds = datetime_to_seconds(&utc, CENTURY_BASE);
  return true;
}

static void request_time_sync(app_t *app) {
  neo6m_data_t data;
  neo6m_get_data(app->gps, &data);
  if (!gps_is_fresh(app) || !data.fix_valid) {
    set_status(app, "Sync: no GPS fix", UI_COLOR_YELLOW);
    return;
  }
  app->sync_pending = true;
  app->sync_deadline = make_timeout_time_ms(GPS_SYNC_TIMEOUT_MS);
  app->refresh_now = true;
}

static void finish_time_sync(app_t *app) {
  neo6m_data_t data;
  neo6m_get_data(app->gps, &data);
  switch (time_sync_rtc_from_gps(app->ds3231, &data)) {
  case TIME_SYNC_OK:
    app->sync_pending = false;
    app->time_invalid = false;
    printf("RTC synced from GPS: %02u:%02u:%02u %02u-%02u-%04u UTC\n",
           data.hours, data.minutes, data.seconds, data.day, data.month,
           CENTURY_BASE + data.year);
    set_status(app, "Time synced", UI_COLOR_GREEN);
    break;
  case TIME_SYNC_LEAP_SECOND:
    break;
  case TIME_SYNC_NO_FIX:
    app->sync_pending = false;
    set_status(app, "Sync: no GPS fix", UI_COLOR_YELLOW);
    break;
  default:
    app->sync_pending = false;
    show_error(app, "RTC write error");
    break;
  }
}

static void check_time_sync_timeout(app_t *app) {
  if (app->sync_pending && time_reached(app->sync_deadline)) {
    app->sync_pending = false;
    set_status(app, "Sync timeout", UI_COLOR_RED);
  }
}

static void gps_store_fix(app_t *app) {
  neo6m_data_t data;
  neo6m_get_data(app->gps, &data);
  if (!data.fix_valid || !data.position_valid) {
    return;
  }
  app->gps_fix = data;
  app->gps_has_fix = true;
  app->gps_fix_utc_valid = gps_utc_seconds(&data, &app->gps_fix_utc_s);
}

static void gps_service(app_t *app) {
  uint32_t updated = NEO6M_UPDATED_NONE;
  if (neo6m_process(app->gps, &updated) != NEO6M_OK) {
    return;
  }
  if (updated != NEO6M_UPDATED_NONE) {
    gps_store_fix(app);
  }
  if (updated & NEO6M_UPDATED_RMC) {
    app->gps_received = true;
    app->gps_last_rmc = get_absolute_time();
    if (app->sync_pending) {
      finish_time_sync(app);
    }
  }
}

static void draw_environment_card(app_t *app) {
  ui_environment_t env = {0};
  env.valid = read_environment(app->sht40, app->bme680, &env.temperature,
                               &env.humidity, &env.pressure);
  if (env.valid) {
    printf("Temperature: %.2f°C, Pressure: %.2f hPa, Humidity: %.2f %%\n",
           env.temperature, env.pressure / 100, env.humidity);
  }

  ds3231_datetime_t datetime;
  ds3231_status_t ds3231_status = ds3231_read_datetime(app->ds3231, &datetime);
  if (ds3231_status != DS3231_OK) {
    printf("Failed to read datetime: %d\n", ds3231_status);
  }
  ui_draw_environment_card(app->oled, &env,
                           (ds3231_status == DS3231_OK) ? &datetime : NULL,
                           !app->time_invalid);
}

static ui_gps_state_t gps_state(const app_t *app, const neo6m_data_t *live) {
  if (!app->gps_received) {
    return UI_GPS_NO_DATA;
  }
  if (!gps_is_fresh(app)) {
    return UI_GPS_TIMEOUT;
  }
  if (live->fix_valid) {
    return UI_GPS_FIX;
  }
  return app->gps_has_fix ? UI_GPS_LOST : UI_GPS_SEARCHING;
}

static bool gps_fix_age(const app_t *app, uint32_t *age_s) {
  if (!app->gps_has_fix || !app->gps_fix_utc_valid || app->time_invalid) {
    return false;
  }
  ds3231_datetime_t now;
  if (ds3231_read_datetime(app->ds3231, &now) != DS3231_OK) {
    return false;
  }
  uint32_t now_s = datetime_to_seconds(&now, CENTURY_BASE);
  if (now_s < app->gps_fix_utc_s) {
    return false;
  }
  *age_s = now_s - app->gps_fix_utc_s;
  return true;
}

static void draw_gps_card(app_t *app) {
  ui_gps_t gps = {0};
  neo6m_get_data(app->gps, &gps.live);
  neo6m_get_stats(app->gps, &gps.stats);
  gps.state = gps_state(app, &gps.live);
  gps.has_fix = app->gps_has_fix;
  gps.fix = app->gps_fix;
  if (gps.state != UI_GPS_FIX) {
    gps.fix_age_valid = gps_fix_age(app, &gps.fix_age_s);
  }
  gps.dropped_bytes = neo6m_port_dropped_bytes();
  ui_draw_gps_card(app->oled, &gps);
}

static const char *card_hint(const app_t *app, uint16_t *color) {
  *color = UI_COLOR_GRAY;
  switch (app->card) {
  case CARD_GPS:
    if (app->sync_pending) {
      *color = UI_COLOR_YELLOW;
      return "Syncing time...";
    }
    return "OK: sync time";
  case CARD_BACKPACK:
    return app->time_invalid ? "" : "OK: start";
  default:
    return "";
  }
}

static void draw_current_card(app_t *app) {
  if (app->clear_screen) {
    ui_clear(app->oled);
    app->clear_screen = false;
  }
  switch (app->card) {
  case CARD_ENVIRONMENT:
    draw_environment_card(app);
    break;
  case CARD_GPS:
    draw_gps_card(app);
    break;
  case CARD_BACKPACK:
    ui_draw_backpack_card(app->oled, !app->time_invalid,
                          BACKPACK_INTERVAL_SECONDS);
    break;
  default:
    break;
  }
  if (status_active(app)) {
    ui_draw_status(app->oled, app->status.text, app->status.color);
  } else {
    uint16_t color;
    const char *hint = card_hint(app, &color);
    ui_draw_status(app->oled, hint, color);
  }
  ui_draw_footer(app->oled, card_titles[app->card], (uint8_t)(app->card + 1),
                 CARD_COUNT);
}

static void enter_backpack_mode(app_t *app) {
  if (app->time_invalid) {
    set_status(app, "Set time via GPS", UI_COLOR_RED);
  } else if (restart_backpack_alarm(app->ds3231)) {
    app->mode = MODE_BACKPACK;
    app->sync_pending = false;
    display_backpack_screen(app->oled, app->ds3231);
  } else {
    show_error(app, "RTC alarm error");
  }
}

static void leave_backpack_mode(app_t *app) {
  app->mode = MODE_MANUAL;
  app->clear_screen = true;
  buttons_take_events();
}

static void handle_back(app_t *app) {
  if (app->sync_pending) {
    app->sync_pending = false;
    set_status(app, "Sync cancelled", UI_COLOR_YELLOW);
  } else if (app->card != CARD_ENVIRONMENT) {
    app->card = CARD_ENVIRONMENT;
    app->clear_screen = true;
  }
}

static void handle_ok(app_t *app) {
  switch (app->card) {
  case CARD_GPS:
    request_time_sync(app);
    break;
  case CARD_BACKPACK:
    buttons_wait_release(BUTTON_OK);
    enter_backpack_mode(app);
    break;
  default:
    break;
  }
}

static void handle_manual_buttons(app_t *app, uint32_t pressed) {
  if (pressed & BUTTON_MASK(BUTTON_BACK)) {
    handle_back(app);
  }
  if (pressed & BUTTON_MASK(BUTTON_OK)) {
    handle_ok(app);
  }
  if (app->mode != MODE_MANUAL) {
    return;
  }
  if (pressed & BUTTON_MASK(BUTTON_NEXT_CARD)) {
    app->card = (card_t)((app->card + 1) % CARD_COUNT);
    app->clear_screen = true;
  }
  if (pressed & BUTTON_MASK(BUTTON_PREV_CARD)) {
    app->card = (card_t)((app->card + CARD_COUNT - 1) % CARD_COUNT);
    app->clear_screen = true;
  }
}

static void run_backpack_cycle(app_t *app) {
  uint32_t wake =
      power_dormant_until_pins(RTC_INT_PIN, buttons_pin(BUTTON_BACK));
  neo6m_port_hw_init();
#ifdef WAKE_DEBUG
  display_wake_debug(app->oled, wake);
#endif

  bool alarm_triggered = false;
  if (ds3231_is_alarm1_triggered(app->ds3231, &alarm_triggered) != DS3231_OK) {
    leave_backpack_mode(app);
    show_error(app, "RTC read error");
    return;
  }

  if (alarm_triggered) {
    take_sample(app->sht40, app->bme680, app->ds3231);
    ds3231_datetime_t current_time;
    if (ds3231_read_datetime(app->ds3231, &current_time) == DS3231_OK) {
      display_backpack_clock(app->oled, &current_time);
    }
    if (!restart_backpack_alarm(app->ds3231)) {
      leave_backpack_mode(app);
      show_error(app, "RTC alarm error");
      return;
    }
  }

  if (wake & POWER_WAKE_BUTTON) {
    buttons_wait_release(BUTTON_BACK);
    leave_backpack_mode(app);
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

  buttons_init();

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

  sht40_t *sht40 =
      sht40_create(I2C_PORT, SHT40_I2C_ADDRESS, sht40_port_i2c_write,
                   sht40_port_i2c_read, sht40_port_delay_ms);
  if (sht40 == NULL) {
    printf("Failed to create SHT40 instance\n");
    return -1;
  }
  sht40_status_t sht40_status = sht40_soft_reset(sht40);
  if (sht40_status != SHT40_OK) {
    printf("SHT40 soft reset failed: %d\n", sht40_status);
  }
  uint32_t sht40_serial = 0;
  sht40_status = sht40_read_serial(sht40, &sht40_serial);
  if (sht40_status != SHT40_OK) {
    printf("Failed to read SHT40 serial: %d\n", sht40_status);
  } else {
    printf("SHT40 serial: 0x%08lX\n", (unsigned long)sht40_serial);
  }

  ssd1351_port_hw_init();
  ssd1351_io_t io = ssd1351_port_get_io();
  ssd1351_t *oled = ssd1351_get_instance();
  ssd1351_init(oled, &io);
  ssd1351_set_rotation(oled, SSD1351_ROT_0);
  ssd1351_fill(oled, 0x0000);

  neo6m_port_hw_init();
  neo6m_t *gps = neo6m_create(NULL, neo6m_port_uart_read);
  if (gps == NULL) {
    printf("Failed to create NEO-6M instance\n");
    return -1;
  }

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
  }

  ds3231_enable_alarm1_interrupt(ds3231, true);
  ds3231_clear_alarm1_flag(ds3231);

  app.oled = oled;
  app.bme680 = bme680;
  app.sht40 = sht40;
  app.ds3231 = ds3231;
  app.gps = gps;
  app.mode = MODE_MANUAL;
  app.card = CARD_ENVIRONMENT;
  app.time_invalid = time_invalid;
  app.clear_screen = true;

  absolute_time_t next_refresh = get_absolute_time();
  while (true) {
    if (app.mode == MODE_MANUAL) {
      gps_service(&app);
      check_time_sync_timeout(&app);
      handle_manual_buttons(&app, buttons_take_events());
      if (app.mode != MODE_MANUAL) {
        continue;
      }

      if (app.clear_screen || app.refresh_now || time_reached(next_refresh)) {
        app.refresh_now = false;
        draw_current_card(&app);
        next_refresh = make_timeout_time_ms(MANUAL_REFRESH_MS);
      }

      while (!buttons_has_events() && !app.refresh_now &&
             !best_effort_wfe_or_timeout(next_refresh)) {
        gps_service(&app);
      }
    } else {
      run_backpack_cycle(&app);
    }
  }
}