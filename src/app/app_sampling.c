#include "app.h"
#include "board_config.h"
#include "datetime_utils.h"
#include "log.h"
#include "sample.h"
#include "sd_log.h"
#include <stdio.h>

#define ENV_SAMPLE_MAX_AGE_MS 1500

// ---------------------------------------------------------------------------
// Sample schedule (DS3231 alarm 1, same in both modes)
// ---------------------------------------------------------------------------

static uint32_t to_seconds(const ds3231_datetime_t *datetime) {
  return datetime_to_seconds(datetime, CENTURY_BASE);
}

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

bool schedule_from_now(app_t *app) {
  ds3231_datetime_t now;
  if (ds3231_read_datetime(app->ds3231, &now) != DS3231_OK) {
    app->schedule_valid = false;
    return false;
  }
  uint32_t offset =
      SAMPLE_INTERVAL_SECONDS - (to_seconds(&now) % SAMPLE_INTERVAL_SECONDS);
  app->next_sample = now;
  datetime_add_seconds(&app->next_sample, offset, CENTURY_BASE);
  if (!arm_sample_alarm(app, false)) {
    app->schedule_valid = false;
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Sampling and logging
// ---------------------------------------------------------------------------

static bool read_climate(sht40_t *sht40, float *temperature, float *humidity) {
  static bool failing = false;
  sht40_status_t status =
      sht40_measure(sht40, SHT40_PRECISION_HIGH, temperature, humidity);
  if (status != SHT40_OK) {
    if (!failing) {
      LOG("SHT40 measurement failed: %d\n", status);
    }
    failing = true;
    return false;
  }
  failing = false;
  return true;
}

static bool read_pressure(bme680_t *bme680, float *pressure) {
  static bool failing = false;
  if (bme680 == NULL) {
    return false;
  }
  float temperature, humidity;
  bme680_status_t status =
      bme680_read_measurement_forced(bme680, &temperature, pressure, &humidity);
  if (status != BME680_OK) {
    if (!failing) {
      LOG("BME680 measurement failed: %d\n", status);
    }
    failing = true;
    return false;
  }
  failing = false;
  return true;
}

bool env_climate(app_t *app, uint32_t max_age_ms, float *temperature,
                 float *humidity) {
  if (!app->climate_seen || ms_since(app->climate_at) > (int64_t)max_age_ms) {
    app->climate_valid = read_climate(app->sht40, &app->climate_temperature_c,
                                      &app->climate_humidity_pct);
    app->climate_at = get_absolute_time();
    app->climate_seen = true;
  }
  *temperature = app->climate_temperature_c;
  *humidity = app->climate_humidity_pct;
  return app->climate_valid;
}

bool env_pressure(app_t *app, uint32_t max_age_ms, float *pressure) {
  if (!app->pressure_seen || ms_since(app->pressure_at) > (int64_t)max_age_ms) {
    app->pressure_valid = read_pressure(app->bme680, &app->pressure_pa);
    app->pressure_at = get_absolute_time();
    app->pressure_seen = true;
  }
  *pressure = app->pressure_pa;
  return app->pressure_valid;
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
  s.mode =
      (app->mode == MODE_BACKPACK) ? SAMPLE_MODE_BACKPACK : SAMPLE_MODE_MANUAL;
  bool gps_data = (app->mode == MODE_BACKPACK) ? gps_wait_for_epoch(app)
                                               : gps_is_fresh(app);
  uint32_t max_age_ms =
      (app->mode == MODE_BACKPACK) ? 0 : ENV_SAMPLE_MAX_AGE_MS;
  s.climate_valid =
      env_climate(app, max_age_ms, &s.temperature_c, &s.humidity_pct);
  s.pressure_valid = env_pressure(app, max_age_ms, &s.pressure_pa);
  fill_gps(app, &s, gps_data);

  char line[SAMPLE_CSV_MAX];
  size_t length = sample_to_csv(&s, line, sizeof(line));
  if (length == 0) {
    return;
  }
  if (app->mode == MODE_MANUAL) {
    LOG("%s", line);
  }
  sd_log_append(line, length, &s.utc, s.time_valid, to_seconds(&s.utc));
}

bool handle_sample_alarm(app_t *app) {
  bool triggered = false;
  if (ds3231_is_alarm1_triggered(app->ds3231, &triggered) != DS3231_OK) {
    return false;
  }
  if (!triggered) {
    return true;
  }
  ds3231_datetime_t scheduled = app->next_sample;
  if (!arm_sample_alarm(app, true)) {
    return false;
  }
  take_sample(app, &scheduled);
  return true;
}

bool rtc_alarm_pending(void) { return !gpio_get(RTC_INT_PIN); }
