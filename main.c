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
#include "sample.h"
#include "sd_log.h"
#include "time_sync.h"
#include "timezone.h"
#include "ui.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#define MANUAL_REFRESH_MS 1000
// Moves the refresh phase into the middle of the RTC second when the shown
// seconds stop advancing by exactly one.
#define REFRESH_PHASE_SHIFT_MS 500
#define SECONDS_PER_MINUTE 60u
#define GPS_STALE_MS 3000
// Power Save Mode: without a fix the receiver may stay silent for up to the
// search period (10 s) - only report a timeout after a longer silence.
#define GPS_IDLE_MS 15000
#define STATUS_MESSAGE_MS 4000
#define GPS_SYNC_TIMEOUT_MS 2500
#define GPS_OFF_GRACE_MS 3000
#define GPS_START_TIMEOUT_MS 5000
// u-blox 6 Receiver Description 9.3.3.1: 0xFF, ~0.5 s, then the command.
#define GPS_WAKE_DELAY_MS 500
// Backpack sample: wait for the next RMC, then for the GGA of that epoch.
#define GPS_SAMPLE_RMC_MS 1200
#define GPS_SAMPLE_GGA_MS 400
#define TIME_INFO_POLL_MS 30000
#define BURN_IN_SHIFT_MS 60000
#define BURN_IN_STEP_PX 2
#define BACKPACK_VIEW_MS 2000
#define GPS_AUTO_SYNC_DEFAULT false

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
  CARD_POSITION,
  CARD_SATELLITES,
  CARD_BACKPACK,
  CARD_COUNT,
} card_t;

static const char *const card_titles[CARD_COUNT] = {
    [CARD_ENVIRONMENT] = "ENV",
    [CARD_POSITION] = "POS",
    [CARD_SATELLITES] = "SAT",
    [CARD_BACKPACK] = "PACK",
};

typedef enum {
  GPS_CMD_CONFIGURE = 1u << 0,
  GPS_CMD_POLL_TIME = 1u << 1,
  GPS_CMD_POWER_OFF = 1u << 2,
} gps_command_t;

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
  bool gps_enabled;
  absolute_time_t gps_toggled_at;
  bool gps_received;
  absolute_time_t gps_last_rmc;
  bool gps_has_fix;
  neo6m_data_t gps_fix;
  bool gps_fix_utc_valid;
  uint32_t gps_fix_utc_s;
  uint32_t gps_commands;
  bool gps_command_woken;
  absolute_time_t gps_command_at;
  bool sync_pending;
  absolute_time_t sync_deadline;
  bool auto_sync;
  bool auto_sync_done;
  absolute_time_t time_info_poll_at;
  ds3231_datetime_t next_sample;
  bool schedule_valid;
  uint32_t shift_step;
  absolute_time_t shift_at;
  bool view_active;
  absolute_time_t view_deadline;
  bool shown_second_valid;
  uint8_t shown_second;
  bool refresh_resync;
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

static int64_t ms_since(absolute_time_t since) {
  return absolute_time_diff_us(since, get_absolute_time()) / 1000;
}

// ---------------------------------------------------------------------------
// Sample schedule (DS3231 alarm 1, same in both modes)
// ---------------------------------------------------------------------------

static uint32_t to_seconds(const ds3231_datetime_t *datetime) {
  return datetime_to_seconds(datetime, CENTURY_BASE);
}

// Arms alarm 1 for app->next_sample, moving it forward until it is still in
// the future after the write: an alarm written for a second that has
// already started would only match again a month later.
static bool arm_sample_alarm(app_t *app, bool advance_first) {
  ds3231_datetime_t now;
  if (ds3231_read_datetime(app->ds3231, &now) != DS3231_OK) {
    return false;
  }
  if (advance_first) {
    datetime_add_seconds(&app->next_sample, SAMPLE_INTERVAL_SECONDS,
                         CENTURY_BASE);
  }
  while (true) {
    while (to_seconds(&app->next_sample) <= to_seconds(&now)) {
      datetime_add_seconds(&app->next_sample, SAMPLE_INTERVAL_SECONDS,
                           CENTURY_BASE);
    }
    const ds3231_datetime_t *n = &app->next_sample;
    if (ds3231_clear_alarm1_flag(app->ds3231) != DS3231_OK ||
        ds3231_set_alarm_1(app->ds3231, n->hours, n->minutes, n->seconds,
                           n->day, DS3231_ALARM1_MATCH_DATE) != DS3231_OK ||
        ds3231_read_datetime(app->ds3231, &now) != DS3231_OK) {
      return false;
    }
    if (to_seconds(&app->next_sample) > to_seconds(&now)) {
      app->schedule_valid = true;
      return true;
    }
  }
}

// Starts the schedule on the interval grid (e.g. :00, :10, :20 ...).
static bool schedule_from_now(app_t *app) {
  ds3231_datetime_t now;
  if (ds3231_read_datetime(app->ds3231, &now) != DS3231_OK) {
    app->schedule_valid = false;
    return false;
  }
  uint32_t offset =
      SAMPLE_INTERVAL_SECONDS - (to_seconds(&now) % SAMPLE_INTERVAL_SECONDS);
  app->next_sample = now;
  datetime_add_seconds(&app->next_sample, (int)offset, CENTURY_BASE);
  if (!arm_sample_alarm(app, false)) {
    app->schedule_valid = false;
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// GPS
// ---------------------------------------------------------------------------

static bool gps_is_fresh(const app_t *app) {
  return app->gps_received && ms_since(app->gps_last_rmc) < GPS_STALE_MS;
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

// Commands to a receiver in Power Save Mode: wake it with 0xFF, wait ~0.5 s,
// then send everything queued (u-blox 6 Receiver Description 9.3.3.1).
static void gps_request(app_t *app, gps_command_t command) {
  if (command == GPS_CMD_POWER_OFF) {
    app->gps_commands &= ~(uint32_t)(GPS_CMD_CONFIGURE | GPS_CMD_POLL_TIME);
  } else if (command == GPS_CMD_CONFIGURE) {
    app->gps_commands &= ~(uint32_t)GPS_CMD_POWER_OFF;
  }
  app->gps_commands |= command;
}

static void gps_send_commands(app_t *app) {
  neo6m_status_t status = NEO6M_OK;
  if (app->gps_commands & GPS_CMD_CONFIGURE) {
    status = neo6m_configure_power_save(app->gps);
  }
  if (status == NEO6M_OK && (app->gps_commands & GPS_CMD_POLL_TIME)) {
    status = neo6m_poll_time_info(app->gps);
  }
  if (status == NEO6M_OK && (app->gps_commands & GPS_CMD_POWER_OFF)) {
    status = neo6m_power_off(app->gps);
  }
  if (status != NEO6M_OK) {
    show_error(app, "GPS UART error");
  }
  app->gps_commands = 0;
  app->gps_command_woken = false;
}

static void gps_commands_service(app_t *app) {
  if (app->gps_commands == 0) {
    return;
  }
  if (!app->gps_command_woken) {
    neo6m_wake(app->gps);
    app->gps_command_woken = true;
    app->gps_command_at = make_timeout_time_ms(GPS_WAKE_DELAY_MS);
    return;
  }
  if (time_reached(app->gps_command_at)) {
    gps_send_commands(app);
  }
}

// Used before going dormant: nothing may stay queued.
static void gps_commands_finish(app_t *app) {
  if (app->gps_commands == 0) {
    return;
  }
  if (!app->gps_command_woken) {
    neo6m_wake(app->gps);
    app->gps_command_at = make_timeout_time_ms(GPS_WAKE_DELAY_MS);
  }
  sleep_until(app->gps_command_at);
  gps_send_commands(app);
}

static void start_time_sync(app_t *app) {
  app->sync_pending = true;
  app->sync_deadline = make_timeout_time_ms(GPS_SYNC_TIMEOUT_MS);
  app->refresh_now = true;
}

static bool gps_leap_seconds_trusted(const neo6m_data_t *data) {
  return data->time_info_valid && !(data->leap_valid && data->leap_default);
}

static void arm_auto_sync(app_t *app) {
  app->auto_sync_done = false;
  app->time_info_poll_at = get_absolute_time();
}

static void toggle_auto_sync(app_t *app) {
  app->auto_sync = !app->auto_sync;
  if (app->auto_sync) {
    arm_auto_sync(app);
  } else if (!app->time_invalid) {
    app->sync_pending = false;
  }
  printf("Auto sync %s\n", app->auto_sync ? "on" : "off");
  set_status(app, app->auto_sync ? "Auto sync ON" : "Auto sync OFF",
             UI_COLOR_GRAY);
}

static void finish_time_sync(app_t *app) {
  neo6m_data_t data;
  neo6m_get_data(app->gps, &data);
  switch (time_sync_rtc_from_gps(app->ds3231, &data)) {
  case TIME_SYNC_OK:
    app->sync_pending = false;
    app->time_invalid = false;
    if (gps_leap_seconds_trusted(&data)) {
      app->auto_sync_done = true;
    }
    printf("RTC synced from GPS: %02u:%02u:%02u %02u-%02u-%04u UTC\n",
           data.hours, data.minutes, data.seconds, data.day, data.month,
           CENTURY_BASE + data.year);
    set_status(app, "Time synced", UI_COLOR_GREEN);
    // The RTC may have jumped: restart the sample schedule from the new time.
    if (!schedule_from_now(app)) {
      show_error(app, "RTC alarm error");
    }
    break;
  case TIME_SYNC_LEAP_SECOND:
    break;
  case TIME_SYNC_NO_FIX:
    app->sync_pending = false;
    break;
  default:
    app->sync_pending = false;
    app->auto_sync_done = true;
    show_error(app, "RTC write error");
    break;
  }
}

static void check_time_sync_timeout(app_t *app) {
  if (app->sync_pending && time_reached(app->sync_deadline)) {
    app->sync_pending = false;
  }
}

static bool auto_sync_wanted(const app_t *app) {
  return app->time_invalid || (app->auto_sync && !app->auto_sync_done);
}

static void auto_sync_check(app_t *app) {
  neo6m_data_t data;
  neo6m_get_data(app->gps, &data);
  if (app->sync_pending || !auto_sync_wanted(app) || !data.fix_valid ||
      !data.time_valid || !data.date_valid) {
    return;
  }
  if (app->time_invalid) {
    start_time_sync(app);
    return;
  }
  if (time_reached(app->time_info_poll_at)) {
    gps_request(app, GPS_CMD_POLL_TIME);
    app->time_info_poll_at = make_timeout_time_ms(TIME_INFO_POLL_MS);
    return;
  }
  if (gps_leap_seconds_trusted(&data)) {
    start_time_sync(app);
  }
}

static const char *auto_sync_hint(const app_t *app, uint16_t *color) {
  if (!app->auto_sync) {
    return "Auto sync: off";
  }
  if (app->auto_sync_done) {
    *color = UI_COLOR_GREEN;
    return "Auto sync: done";
  }
  *color = UI_COLOR_YELLOW;
  neo6m_data_t live;
  neo6m_get_data(app->gps, &live);
  if (gps_is_fresh(app) && live.fix_valid && live.time_info_valid &&
      live.leap_valid && live.leap_default) {
    return "Auto sync: wait";
  }
  return "Auto sync: on";
}

static void gps_set_enabled(app_t *app, bool enabled) {
  if (enabled && neo6m_power_on(app->gps) != NEO6M_OK) {
    show_error(app, "GPS UART error");
    return;
  }
  gps_request(app, enabled ? GPS_CMD_CONFIGURE : GPS_CMD_POWER_OFF);
  app->gps_enabled = enabled;
  app->gps_received = false;
  app->gps_toggled_at = get_absolute_time();
  app->sync_pending = false;
  if (enabled) {
    arm_auto_sync(app);
  }
  printf("GPS %s\n", enabled ? "on" : "off");
  set_status(app, enabled ? "GPS on" : "GPS off", UI_COLOR_GRAY);
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

// Returns the NEO6M_UPDATED_* flags of this call.
static uint32_t gps_service(app_t *app) {
  uint32_t updated = NEO6M_UPDATED_NONE;
  if (neo6m_process(app->gps, &updated) != NEO6M_OK) {
    return NEO6M_UPDATED_NONE;
  }
  if (!app->gps_enabled) {
    if (!(updated & NEO6M_UPDATED_RMC) || app->gps_commands != 0 ||
        ms_since(app->gps_toggled_at) < GPS_OFF_GRACE_MS) {
      return updated;
    }
    app->gps_enabled = true;
    app->gps_toggled_at = get_absolute_time();
    show_error(app, "GPS off failed");
  }
  if (updated != NEO6M_UPDATED_NONE) {
    gps_store_fix(app);
  }
  if (updated & NEO6M_UPDATED_RMC) {
    app->gps_received = true;
    app->gps_last_rmc = get_absolute_time();
    if (app->mode == MODE_MANUAL) {
      if (app->sync_pending) {
        finish_time_sync(app);
      } else {
        auto_sync_check(app);
      }
    }
  }
  return updated;
}

// After a dormant period the system timer has not advanced, so "fresh" is
// meaningless: wait for an RMC (and the GGA of the same epoch) that really
// arrived after the wake-up. Returns true if an RMC was received.
static bool gps_wait_for_epoch(app_t *app) {
  if (!app->gps_enabled) {
    return false;
  }
  bool rmc = false;
  absolute_time_t deadline = make_timeout_time_ms(GPS_SAMPLE_RMC_MS);
  while (true) {
    uint32_t updated = gps_service(app);
    if (updated & NEO6M_UPDATED_RMC) {
      rmc = true;
      deadline = make_timeout_time_ms(GPS_SAMPLE_GGA_MS);
    }
    if ((rmc && (updated & NEO6M_UPDATED_GGA)) ||
        best_effort_wfe_or_timeout(deadline)) {
      return rmc;
    }
  }
}

// ---------------------------------------------------------------------------
// Sampling and logging
// ---------------------------------------------------------------------------

static bool read_climate(sht40_t *sht40, float *temperature, float *humidity) {
  sht40_status_t status =
      sht40_measure(sht40, SHT40_PRECISION_HIGH, temperature, humidity);
  if (status != SHT40_OK) {
    printf("SHT40 measurement failed: %d\n", status);
    return false;
  }
  return true;
}

static bool read_pressure(bme680_t *bme680, float *pressure) {
  float temperature, humidity;
  bme680_status_t status = bme680_read_measurement_forced(
      bme680, &temperature, pressure, &humidity);
  if (status != BME680_OK) {
    printf("BME680 measurement failed: %d\n", status);
    return false;
  }
  return true;
}

static void fill_gps(const app_t *app, sample_t *s, bool data_received) {
  if (!app->gps_enabled) {
    s->gps = SAMPLE_GPS_OFF;
    return;
  }
  if (!data_received) {
    s->gps = SAMPLE_GPS_NO_DATA;
    return;
  }
  neo6m_data_t live;
  neo6m_get_data(app->gps, &live);
  s->satellites = live.satellites;
  if (!live.fix_valid || !live.position_valid) {
    s->gps = SAMPLE_GPS_NO_FIX;
    return;
  }
  s->gps = SAMPLE_GPS_FIX;
  s->latitude_e7 = live.latitude_e7;
  s->longitude_e7 = live.longitude_e7;
  s->altitude_valid = live.altitude_valid;
  s->altitude_dm = live.altitude_dm;
  s->speed_valid = live.speed_valid;
  s->speed_kmh_x10 = live.speed_kmh_x10;
  s->course_valid = live.course_valid;
  s->course_x100 = live.course_x100;
  s->hdop_valid = live.hdop_valid;
  s->hdop_x100 = live.hdop_x100;
}

static void take_sample(app_t *app, const ds3231_datetime_t *scheduled) {
  sample_t s = {0};
  s.utc = *scheduled;
  s.time_valid = !app->time_invalid;
  s.mode = (app->mode == MODE_BACKPACK) ? SAMPLE_MODE_BACKPACK
                                        : SAMPLE_MODE_MANUAL;
  bool gps_data = (app->mode == MODE_BACKPACK) ? gps_wait_for_epoch(app)
                                               : gps_is_fresh(app);
  s.climate_valid = read_climate(app->sht40, &s.temperature_c, &s.humidity_pct);
  s.pressure_valid = read_pressure(app->bme680, &s.pressure_pa);
  fill_gps(app, &s, gps_data);

  char line[SAMPLE_CSV_MAX];
  size_t length = sample_to_csv(&s, line, sizeof(line));
  if (length == 0) {
    return;
  }
  printf("%s", line);
  sd_log_append(line, length, &s.utc, s.time_valid, to_seconds(&s.utc));
}

// Returns false on an RTC error.
static bool handle_sample_alarm(app_t *app) {
  bool triggered = false;
  if (ds3231_is_alarm1_triggered(app->ds3231, &triggered) != DS3231_OK) {
    return false;
  }
  if (!triggered) {
    return true;
  }
  ds3231_datetime_t scheduled = app->next_sample;
  // Clear A1F and arm the next alarm before measuring (INT released early).
  if (!arm_sample_alarm(app, true)) {
    return false;
  }
  take_sample(app, &scheduled);
  return true;
}

static bool rtc_alarm_pending(void) { return !gpio_get(RTC_INT_PIN); }

// ---------------------------------------------------------------------------
// Manual mode UI
// ---------------------------------------------------------------------------

// Called for scheduled (1 s) refreshes only: the shown second must advance by
// exactly one, otherwise the refresh sits on the RTC second boundary.
static void track_shown_second(app_t *app, uint8_t second) {
  if (app->shown_second_valid) {
    uint8_t step = (uint8_t)((second + SECONDS_PER_MINUTE - app->shown_second) %
                             SECONDS_PER_MINUTE);
    if (step != 1) {
      app->refresh_resync = true;
    }
  }
  app->shown_second = second;
  app->shown_second_valid = true;
}

static void draw_environment_card(app_t *app, bool scheduled) {
  // RTC first: its read time relative to the refresh tick must not depend on
  // how long the sensor measurements take.
  ds3231_datetime_t datetime;
  ds3231_status_t ds3231_status = ds3231_read_datetime(app->ds3231, &datetime);
  if (ds3231_status != DS3231_OK) {
    printf("Failed to read datetime: %d\n", ds3231_status);
    app->shown_second_valid = false;
  } else if (scheduled) {
    track_shown_second(app, datetime.seconds);
  }

  ui_environment_t env = {0};
  env.climate_valid =
      read_climate(app->sht40, &env.temperature, &env.humidity);
  env.pressure_valid = read_pressure(app->bme680, &env.pressure);

  ui_draw_environment_card(app->oled, &env,
                           (ds3231_status == DS3231_OK) ? &datetime : NULL,
                           !app->time_invalid);
}

static ui_gps_state_t gps_state(const app_t *app, const neo6m_data_t *live) {
  if (!app->gps_enabled) {
    return UI_GPS_OFF;
  }
  if (!app->gps_received) {
    return (ms_since(app->gps_toggled_at) < GPS_START_TIMEOUT_MS)
               ? UI_GPS_STARTING
               : UI_GPS_NO_DATA;
  }
  if (!gps_is_fresh(app)) {
    return (ms_since(app->gps_last_rmc) < GPS_IDLE_MS) ? UI_GPS_IDLE
                                                         : UI_GPS_TIMEOUT;
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

static void collect_gps(app_t *app, ui_gps_t *gps) {
  neo6m_get_data(app->gps, &gps->live);
  neo6m_get_stats(app->gps, &gps->stats);
  gps->state = gps_state(app, &gps->live);
  gps->has_fix = app->gps_has_fix;
  gps->fix = app->gps_fix;
  if (gps->state != UI_GPS_FIX) {
    gps->fix_age_valid = gps_fix_age(app, &gps->fix_age_s);
  }
  gps->dropped_bytes = neo6m_port_dropped_bytes();
}

static const char *card_hint(const app_t *app, uint16_t *color) {
  *color = UI_COLOR_GRAY;
  switch (app->card) {
  case CARD_POSITION:
    return app->gps_enabled ? "" : "Hold BACK: on";
  case CARD_SATELLITES:
    if (!app->gps_enabled) {
      return "Hold BACK: on";
    }
    if (app->sync_pending) {
      *color = UI_COLOR_YELLOW;
      return "Syncing time...";
    }
    return auto_sync_hint(app, color);
  case CARD_BACKPACK:
    return app->time_invalid ? "" : "OK: start";
  default:
    return "";
  }
}

static uint16_t gps_tag_color(const app_t *app) {
  neo6m_data_t live;
  neo6m_get_data(app->gps, &live);
  switch (gps_state(app, &live)) {
  case UI_GPS_FIX:
    return UI_COLOR_GREEN;
  case UI_GPS_STARTING:
  case UI_GPS_SEARCHING:
  case UI_GPS_LOST:
  case UI_GPS_IDLE:
    return UI_COLOR_YELLOW;
  default:
    return UI_COLOR_RED;
  }
}

static uint16_t sd_tag_color(void) {
  switch (sd_log_state()) {
  case SD_LOG_READY:
    return UI_COLOR_GREEN;
  case SD_LOG_ERROR:
    return UI_COLOR_RED;
  case SD_LOG_NO_CARD:
  default:
    return UI_COLOR_GRAY;
  }
}

static void draw_footer(app_t *app) {
  ui_tag_t tags[2];
  uint8_t count = 0;
  tags[count++] = (ui_tag_t){.text = "SD", .color = sd_tag_color()};
  bool gps_card =
      app->card == CARD_POSITION || app->card == CARD_SATELLITES;
  if (app->gps_enabled && !gps_card) {
    tags[count++] = (ui_tag_t){.text = "GPS", .color = gps_tag_color(app)};
  }
  ui_draw_footer(app->oled, card_titles[app->card], (uint8_t)(app->card + 1),
                 CARD_COUNT, tags, count);
}

static void draw_current_card(app_t *app, bool scheduled) {
  if (app->clear_screen) {
    ui_clear(app->oled);
    app->clear_screen = false;
  }
  if (app->card != CARD_ENVIRONMENT) {
    app->shown_second_valid = false;
  }
  ui_gps_t gps = {0};
  switch (app->card) {
  case CARD_ENVIRONMENT:
    draw_environment_card(app, scheduled);
    break;
  case CARD_POSITION:
    collect_gps(app, &gps);
    ui_draw_position_card(app->oled, &gps);
    break;
  case CARD_SATELLITES:
    collect_gps(app, &gps);
    ui_draw_satellite_card(app->oled, &gps);
    break;
  case CARD_BACKPACK:
    ui_draw_backpack_card(app->oled, !app->time_invalid,
                          SAMPLE_INTERVAL_SECONDS);
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
  draw_footer(app);
}

// Moves the whole layout a few pixels every minute (OLED burn-in).
static void apply_shift(const app_t *app) {
  uint8_t steps_x = UI_SHIFT_MAX_X / BURN_IN_STEP_PX + 1;
  uint8_t steps_y = UI_SHIFT_MAX_Y / BURN_IN_STEP_PX + 1;
  uint8_t dx = (uint8_t)((app->shift_step % steps_x) * BURN_IN_STEP_PX);
  uint8_t dy =
      (uint8_t)(((app->shift_step / steps_x) % steps_y) * BURN_IN_STEP_PX);
  ui_set_shift(dx, dy);
}

static void update_shift(app_t *app) {
  if (!time_reached(app->shift_at)) {
    return;
  }
  app->shift_step++;
  app->shift_at = make_timeout_time_ms(BURN_IN_SHIFT_MS);
  apply_shift(app);
  app->clear_screen = true;
}

// ---------------------------------------------------------------------------
// Backpack mode
// ---------------------------------------------------------------------------

static void show_backpack_view(app_t *app) {
  ds3231_datetime_t now;
  bool ok = ds3231_read_datetime(app->ds3231, &now) == DS3231_OK;
  ssd1351_set_contrast(app->oled, OLED_CONTRAST_BACKPACK);
  ui_draw_backpack_view(app->oled, ok ? &now : NULL,
                        sd_log_state() == SD_LOG_ERROR);
  ssd1351_set_sleep(app->oled, false);
  app->view_active = true;
  app->view_deadline = make_timeout_time_ms(BACKPACK_VIEW_MS);
}

static void hide_backpack_view(app_t *app) {
  ssd1351_set_sleep(app->oled, true);
  app->view_active = false;
}

static void enter_backpack_mode(app_t *app) {
  if (app->time_invalid) {
    set_status(app, "Sync time first", UI_COLOR_RED);
    return;
  }
  if (!app->schedule_valid && !schedule_from_now(app)) {
    show_error(app, "RTC alarm error");
    return;
  }
  gps_commands_finish(app);
  app->mode = MODE_BACKPACK;
  app->sync_pending = false;
  show_backpack_view(app);
}

static void leave_backpack_mode(app_t *app) {
  app->mode = MODE_MANUAL;
  app->view_active = false;
  ssd1351_set_contrast(app->oled, OLED_CONTRAST_MANUAL);
  ssd1351_set_sleep(app->oled, false);
  apply_shift(app);
  app->shift_at = make_timeout_time_ms(BURN_IN_SHIFT_MS);
  app->clear_screen = true;
  app->shown_second_valid = false;
  buttons_take_events();
}

static void abort_backpack_mode(app_t *app, const char *message) {
  leave_backpack_mode(app);
  show_error(app, message);
}

static uint32_t wait_while_view_active(app_t *app) {
  while (!buttons_has_events() && !rtc_alarm_pending() &&
         !best_effort_wfe_or_timeout(app->view_deadline)) {
    gps_service(app);
  }
  if (time_reached(app->view_deadline)) {
    hide_backpack_view(app);
  }
  return buttons_take_events();
}

static uint32_t sleep_until_event(app_t *app) {
  uint8_t ok_pin = buttons_pin(BUTTON_OK);
  uint8_t back_pin = buttons_pin(BUTTON_BACK);
  uint32_t wake = power_dormant_until(
      RTC_INT_PIN, POWER_PIN_MASK(ok_pin) | POWER_PIN_MASK(back_pin));
  neo6m_port_hw_init();

  uint32_t events = 0;
  if (wake & POWER_PIN_MASK(ok_pin)) {
    buttons_wait_release(BUTTON_OK);
    events |= BUTTON_MASK(BUTTON_OK);
  }
  if (wake & POWER_PIN_MASK(back_pin)) {
    buttons_wait_release(BUTTON_BACK);
    events |= BUTTON_MASK(BUTTON_BACK);
  }
  return events;
}

static void run_backpack_cycle(app_t *app) {
  uint32_t events =
      app->view_active ? wait_while_view_active(app) : sleep_until_event(app);

  if (rtc_alarm_pending() && !handle_sample_alarm(app)) {
    abort_backpack_mode(app, "RTC error");
    return;
  }
  if (events & (BUTTON_MASK(BUTTON_BACK) | BUTTON_LONG_MASK(BUTTON_BACK))) {
    leave_backpack_mode(app);
    return;
  }
  if (events & BUTTON_MASK(BUTTON_OK)) {
    show_backpack_view(app);
  }
}

// ---------------------------------------------------------------------------
// Manual mode buttons
// ---------------------------------------------------------------------------

static void handle_back(app_t *app) {
  if (app->card != CARD_ENVIRONMENT) {
    app->card = CARD_ENVIRONMENT;
    app->clear_screen = true;
  }
}

static void handle_ok(app_t *app) {
  switch (app->card) {
  case CARD_SATELLITES:
    toggle_auto_sync(app);
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
  if (pressed & BUTTON_LONG_MASK(BUTTON_BACK)) {
    gps_set_enabled(app, !app->gps_enabled);
  }
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

static void run_manual_cycle(app_t *app, absolute_time_t *next_refresh) {
  gps_service(app);
  gps_commands_service(app);
  check_time_sync_timeout(app);
  if (rtc_alarm_pending() && !handle_sample_alarm(app)) {
    show_error(app, "RTC error");
  }
  handle_manual_buttons(app, buttons_take_events());
  if (app->mode != MODE_MANUAL) {
    return;
  }
  update_shift(app);

  bool scheduled = time_reached(*next_refresh);
  if (app->clear_screen || app->refresh_now || scheduled) {
    app->refresh_now = false;
    draw_current_card(app, scheduled);
  }
  if (scheduled) {
    // Fixed cadence: the next tick does not depend on how long drawing took.
    *next_refresh = delayed_by_ms(*next_refresh, MANUAL_REFRESH_MS);
    if (app->refresh_resync) {
      *next_refresh = delayed_by_ms(*next_refresh, REFRESH_PHASE_SHIFT_MS);
      app->refresh_resync = false;
    }
    if (time_reached(*next_refresh)) {
      // Fell behind (e.g. a long SD write): restart the cadence.
      *next_refresh = make_timeout_time_ms(MANUAL_REFRESH_MS);
      app->shown_second_valid = false;
    }
  }

  while (!buttons_has_events() && !app->refresh_now && !rtc_alarm_pending() &&
         !best_effort_wfe_or_timeout(*next_refresh)) {
    gps_service(app);
    gps_commands_service(app);
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

  buttons_init(BUTTON_MASK(BUTTON_BACK));

  gpio_init(RTC_INT_PIN);
  gpio_set_dir(RTC_INT_PIN, GPIO_IN);
  gpio_pull_up(RTC_INT_PIN);
  // Wakes the manual-mode WFE when an alarm fires; the shared GPIO callback
  // (buttons) ignores this pin.
  gpio_set_irq_enabled(RTC_INT_PIN, GPIO_IRQ_EDGE_FALL, true);

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
  ssd1351_set_contrast(oled, OLED_CONTRAST_MANUAL);
  ssd1351_fill(oled, 0x0000);

  // Shares SPI0 with the OLED, so it is initialised after it.
  sd_log_init();

  neo6m_port_hw_init();
  neo6m_t *gps =
      neo6m_create(NULL, neo6m_port_uart_read, neo6m_port_uart_write);
  if (gps == NULL) {
    printf("Failed to create NEO-6M instance\n");
    return -1;
  }
  if (neo6m_power_on(gps) != NEO6M_OK) {
    printf("Failed to wake NEO-6M\n");
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
  app.gps_enabled = true;
  app.gps_toggled_at = get_absolute_time();
  app.auto_sync = GPS_AUTO_SYNC_DEFAULT;
  app.shift_at = make_timeout_time_ms(BURN_IN_SHIFT_MS);
  gps_request(&app, GPS_CMD_CONFIGURE);
  if (!schedule_from_now(&app)) {
    show_error(&app, "RTC alarm error");
  }

  absolute_time_t next_refresh = get_absolute_time();
  while (true) {
    if (app.mode == MODE_MANUAL) {
      run_manual_cycle(&app, &next_refresh);
    } else {
      run_backpack_cycle(&app);
    }
  }
}