#include "app.h"
#include "board_config.h"
#include "sd_log.h"
#include "sun.h"
#include <stdio.h>

#define SECONDS_PER_MINUTE 60u
#define BURN_IN_STEP_PX 2
#define COMPASS_REFRESH_MS 200
#define COMPASS_CAL_REFRESH_MS 100
#define IMU_REFRESH_MS 200
#define PDR_REFRESH_MS 200
#define TOOLS_REFRESH_MS 500
#define LINE_REFRESH_MS 300

static const char *const card_titles[CARD_COUNT] = {
    [CARD_ENVIRONMENT] = "ENV", [CARD_POSITION] = "POS",
    [CARD_SATELLITES] = "SAT",  [CARD_COMPASS] = "CMP",
    [CARD_NAV] = "NAV",         [CARD_ALT] = "ALT",
    [CARD_TRIP] = "TRIP",       [CARD_PDR] = "PDR",
    [CARD_WPT] = "WPT",         [CARD_DIST] = "DIST",
    [CARD_LINE] = "LINE",       [CARD_AREA] = "AREA",
    [CARD_SUN] = "SUN",         [CARD_IMU] = "IMU",
    [CARD_BACKPACK] = "PACK",
};

// ---------------------------------------------------------------------------
// NAV and ALT cards
// ---------------------------------------------------------------------------

static void draw_nav_card(app_t *app) {
  cards_nav_t c = {
      .source = nav_source(app),
      .sigma_m = fusion_sigma_m(&app->fusion),
      .gps_on = app->gps_enabled,
      .gps_used = app->fusion.updates,
      .gps_rejected = app->fusion.rejected,
      .altitude_valid = app->alt.valid,
      .altitude_gps_ref = app->alt.offset_valid,
      .altitude_m = alt_altitude_m(&app->alt),
      .track_m = app->track_m,
  };
  c.latlon_valid = fusion_lat_lon(&app->fusion, &c.lat_e7, &c.lon_e7);
  if (app->gps_enabled && gps_is_fresh(app)) {
    neo6m_data_t live;
    neo6m_get_data(app->gps, &live);
    c.gps_fix = live.fix_valid && live.position_valid;
    c.satellites = live.satellites;
    c.hdop = live.hdop_valid ? (float)live.hdop_x100 / 100.0f : 99.9f;
  }
  c.heading_valid = walking_heading(app, &c.heading_deg, NULL);
  ui_row_t rows[UI_CONTENT_ROWS];
  cards_nav(&c, rows);
  ui_draw_rows(app->oled, rows);
}

static void draw_alt_card(app_t *app) {
  cards_alt_t c = {
      .state = ui_trip_state(&app->trip),
      .valid = app->alt.valid,
      .altitude_m = alt_altitude_m(&app->alt),
      .gps_ref = app->alt.offset_valid,
      .pressure_hpa = app->alt.pressure_pa / 100.0f,
      .ascent_m = app->alt.ascent_m,
      .descent_m = app->alt.descent_m,
      .horizontal_m = app->trip.distance_m,
      .slope_m = app->trip.distance_m + app->alt.slope_extra_m,
      .threshold_m = app->alt.config.threshold_m,
  };
  ui_row_t rows[UI_CONTENT_ROWS];
  cards_alt(&c, rows);
  ui_draw_rows(app->oled, rows);
}

// ---------------------------------------------------------------------------
// SUN card
// ---------------------------------------------------------------------------

static bool sun_position_source(app_t *app, double *lat, double *lon,
                                const char **source) {
  int32_t lat_e7, lon_e7;
  if (fusion_lat_lon(&app->fusion, &lat_e7, &lon_e7)) {
    *source = "GPS+steps";
  } else if (app->gps_has_fix) {
    lat_e7 = app->gps_fix.latitude_e7;
    lon_e7 = app->gps_fix.longitude_e7;
    *source = "last fix";
  } else {
    const waypoint_t *found = NULL;
    for (uint16_t i = app->wps.count; i > 0; i--) {
      if (app->wps.items[i - 1u].flags & WAYPOINT_HAS_LATLON) {
        found = &app->wps.items[i - 1u];
        break;
      }
    }
    if (found == NULL) {
      return false;
    }
    lat_e7 = found->lat_e7;
    lon_e7 = found->lon_e7;
    *source = "waypoint";
  }
  *lat = (double)lat_e7 / 1e7;
  *lon = (double)lon_e7 / 1e7;
  return true;
}

static cards_sun_status_t sun_status(sun_event_status_t status) {
  switch (status) {
  case SUN_EVENT_ALWAYS_UP:
    return CARDS_SUN_ALWAYS_UP;
  case SUN_EVENT_ALWAYS_DOWN:
    return CARDS_SUN_ALWAYS_DOWN;
  default:
    return CARDS_SUN_OK;
  }
}

static void draw_sun_card(app_t *app) {
  cards_sun_t c = {0};
  ds3231_datetime_t utc;
  uint32_t now_s = 0;
  double lat = 0.0, lon = 0.0;
  c.time_valid = rtc_now(app, &utc, &now_s);
  c.position_valid = sun_position_source(app, &lat, &lon, &c.position_source);
  if (c.time_valid && c.position_valid) {
    int32_t offset = local_offset_s(now_s);
    int64_t local_now = (int64_t)now_s + offset;
    int64_t local_noon = local_now - local_now % 86400 + 43200;
    sun_day_t day;
    sun_day(local_noon - offset, lat, lon, &day);
    c.status = sun_status(day.status);
    c.civil_status = sun_status(day.civil_status);
    c.noon_min = local_minutes(day.noon_s);
    if (day.status == SUN_EVENT_OK) {
      c.rise_min = local_minutes(day.rise_s);
      c.set_min = local_minutes(day.set_s);
      c.day_length_min = (uint16_t)((day.set_s - day.rise_s) / 60);
      int64_t target = 0;
      if ((int64_t)now_s < day.rise_s) {
        target = day.rise_s;
      } else if ((int64_t)now_s < day.set_s) {
        target = day.set_s;
        c.to_set = true;
      } else {
        sun_day_t next;
        sun_day(local_noon - offset + 86400, lat, lon, &next);
        if (next.status == SUN_EVENT_OK) {
          target = next.rise_s;
        }
      }
      if (target > (int64_t)now_s) {
        c.countdown_valid = true;
        c.countdown_min = (uint32_t)((target - (int64_t)now_s + 59) / 60);
      }
    }
    if (day.civil_status == SUN_EVENT_OK) {
      c.dawn_min = local_minutes(day.dawn_s);
      c.dusk_min = local_minutes(day.dusk_s);
    }
    double elevation, azimuth;
    sun_position(now_s, lat, lon, &elevation, &azimuth);
    c.elevation_deg = (float)elevation;
    c.azimuth_deg = (float)azimuth;
  }
  ui_row_t rows[UI_CONTENT_ROWS];
  cards_sun(&c, rows);
  ui_draw_rows(app->oled, rows);
}

// ---------------------------------------------------------------------------
// Card rendering
// ---------------------------------------------------------------------------

uint32_t fast_refresh_ms(const app_t *app) {
  switch (app->card) {
  case CARD_COMPASS:
    return app->mag_calibrating ? COMPASS_CAL_REFRESH_MS : COMPASS_REFRESH_MS;
  case CARD_IMU:
    return IMU_REFRESH_MS;
  case CARD_PDR:
    return PDR_REFRESH_MS;
  case CARD_NAV:
  case CARD_WPT:
  case CARD_DIST:
  case CARD_AREA:
    return TOOLS_REFRESH_MS;
  case CARD_LINE:
    return LINE_REFRESH_MS;
  default:
    return 0;
  }
}

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
  ds3231_datetime_t datetime;
  ds3231_status_t ds3231_status = ds3231_read_datetime(app->ds3231, &datetime);
  if (ds3231_status != DS3231_OK) {
    printf("Failed to read datetime: %d\n", ds3231_status);
    app->shown_second_valid = false;
  } else if (scheduled) {
    track_shown_second(app, datetime.seconds);
  }

  ui_environment_t env = {0};
  env.climate_valid = read_climate(app->sht40, &env.temperature, &env.humidity);
  env.pressure_valid = read_pressure(app->bme680, &env.pressure);

  ui_draw_environment_card(app->oled, &env,
                           (ds3231_status == DS3231_OK) ? &datetime : NULL,
                           !app->time_invalid);
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
  case CARD_COMPASS:
    if (!app->mag_ok) {
      return "";
    }
    if (app->mag_calibrating) {
      *color = UI_COLOR_YELLOW;
      return "OK:save BACK:x";
    }
    if (!app->mag_cal.valid) {
      *color = UI_COLOR_YELLOW;
      return "OK: calibrate";
    }
    return "OK: recalibrate";
  case CARD_WPT:
  case CARD_DIST:
  case CARD_LINE:
  case CARD_AREA:
    return tools_hint(app);
  case CARD_TRIP:
  case CARD_PDR:
  case CARD_ALT:
    switch (app->trip.state) {
    case TRIP_RUNNING:
      return "OK: pause";
    case TRIP_PAUSED:
      return "OK:go hold:rst";
    default:
      return "OK: start";
    }
  case CARD_BACKPACK:
    return app->time_invalid ? "" : "OK: start";
  default:
    return "";
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
  bool gps_card = app->card == CARD_POSITION || app->card == CARD_SATELLITES;
  if (app->gps_enabled && !gps_card) {
    tags[count++] = (ui_tag_t){.text = "GPS", .color = gps_tag_color(app)};
  }
  ui_draw_footer(app->oled, card_titles[app->card], (uint8_t)(app->card + 1),
                 CARD_COUNT, tags, count);
}

void draw_current_card(app_t *app, bool scheduled) {
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
  case CARD_COMPASS:
    draw_compass_card(app);
    break;
  case CARD_NAV:
    draw_nav_card(app);
    break;
  case CARD_ALT:
    draw_alt_card(app);
    break;
  case CARD_WPT:
    draw_wpt_card(app);
    break;
  case CARD_DIST:
    draw_dist_card(app);
    break;
  case CARD_LINE:
    draw_line_card(app);
    break;
  case CARD_AREA:
    draw_area_card(app);
    break;
  case CARD_SUN:
    draw_sun_card(app);
    break;
  case CARD_TRIP:
    draw_trip_card(app);
    break;
  case CARD_PDR:
    draw_pdr_card(app);
    break;
  case CARD_IMU:
    draw_imu_card(app);
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
  uint32_t fast_ms = fast_refresh_ms(app);
  if (fast_ms != 0) {
    app->fast_refresh_at = make_timeout_time_ms(fast_ms);
  }
}

// ---------------------------------------------------------------------------
// Burn-in pixel shift
// ---------------------------------------------------------------------------

void apply_shift(const app_t *app) {
  uint8_t steps_x = UI_SHIFT_MAX_X / BURN_IN_STEP_PX + 1;
  uint8_t steps_y = UI_SHIFT_MAX_Y / BURN_IN_STEP_PX + 1;
  uint8_t dx = (uint8_t)((app->shift_step % steps_x) * BURN_IN_STEP_PX);
  uint8_t dy =
      (uint8_t)(((app->shift_step / steps_x) % steps_y) * BURN_IN_STEP_PX);
  ui_set_shift(dx, dy);
}

void update_shift(app_t *app) {
  if (!time_reached(app->shift_at)) {
    return;
  }
  app->shift_step++;
  app->shift_at = make_timeout_time_ms(BURN_IN_SHIFT_MS);
  apply_shift(app);
  app->clear_screen = true;
}
