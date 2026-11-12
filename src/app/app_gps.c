#include "app.h"
#include "board_config.h"
#include "datetime_utils.h"
#include "neo6m_port.h"
#include "time_sync.h"
#include <stdio.h>

#define GPS_STALE_MS 3000
#define GPS_IDLE_MS 15000
#define GPS_SYNC_TIMEOUT_MS 2500
#define GPS_OFF_GRACE_MS 3000
#define GPS_START_TIMEOUT_MS 5000
#define GPS_WAKE_DELAY_MS 500
#define GPS_SAMPLE_RMC_MS 1200
#define GPS_SAMPLE_GGA_MS 400
#define TIME_INFO_POLL_MS 30000

bool gps_is_fresh(const app_t *app) {
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

void gps_request(app_t *app, gps_command_t command) {
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

void gps_commands_service(app_t *app) {
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

void gps_commands_finish(app_t *app) {
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

void toggle_auto_sync(app_t *app) {
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

void check_time_sync_timeout(app_t *app) {
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

const char *auto_sync_hint(const app_t *app, uint16_t *color) {
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

void gps_set_enabled(app_t *app, bool enabled) {
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

uint32_t gps_service(app_t *app) {
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
  if (updated & NEO6M_UPDATED_GGA) {
    nav_gps(app);
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

bool gps_wait_for_epoch(app_t *app) {
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
// GPS cards
// ---------------------------------------------------------------------------

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

void collect_gps(app_t *app, ui_gps_t *gps) {
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

uint16_t gps_tag_color(const app_t *app) {
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
