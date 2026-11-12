#include "app.h"
#include "board_config.h"
#include "datetime_utils.h"
#include "geo.h"
#include "sd_log.h"
#include "wp_flash.h"
#include <math.h>
#include <stdio.h>

static waypoint_t waypoint_items[WP_FLASH_SLOTS];

void waypoints_start(app_t *app) {
  wp_storage_t storage;
  if (!wp_flash_storage(&storage)) {
    printf("Waypoints: firmware overlaps the flash region\n");
    app->wps_ok = false;
    return;
  }
  waypoints_load(&app->wps, &storage, waypoint_items);
  app->wps_ok = true;
  app->wp_index = (app->wps.count > 0) ? (uint16_t)(app->wps.count - 1u) : 0;
  printf("Waypoints: %u loaded, %u damaged, %u free\n", app->wps.count,
         app->wps.damaged,
         (unsigned)(app->wps.storage.slot_count - app->wps.next_slot));
}

const waypoint_t *waypoint_by_id(const app_t *app, uint16_t id) {
  for (uint16_t i = 0; i < app->wps.count; i++) {
    if (app->wps.items[i].id == id) {
      return &app->wps.items[i];
    }
  }
  return NULL;
}

bool waypoint_local(const app_t *app, const waypoint_t *wp,
                           measure_point_t *p) {
  if (wp->session == app->session) {
    *p = (measure_point_t){wp->east_m, wp->north_m};
    return true;
  }
  if ((wp->flags & WAYPOINT_HAS_LATLON) && app->fusion.origin.valid) {
    geo_to_local(&app->fusion.origin, wp->lat_e7, wp->lon_e7, &p->east_m,
                 &p->north_m);
    return true;
  }
  return false;
}

static void format_e7(char *buffer, size_t size, int32_t value_e7) {
  uint32_t magnitude =
      (value_e7 < 0) ? (uint32_t)(-(int64_t)value_e7) : (uint32_t)value_e7;
  snprintf(buffer, size, "%s%lu.%07lu", (value_e7 < 0) ? "-" : "",
           (unsigned long)(magnitude / 10000000u),
           (unsigned long)(magnitude % 10000000u));
}

static void waypoint_log(const waypoint_t *wp, const ds3231_datetime_t *utc,
                         bool time_valid) {
  char lat[16] = "";
  char lon[16] = "";
  char when[32] = "";
  char alt[16] = "";
  if (wp->flags & WAYPOINT_HAS_LATLON) {
    format_e7(lat, sizeof(lat), wp->lat_e7);
    format_e7(lon, sizeof(lon), wp->lon_e7);
  }
  if (time_valid) {
    snprintf(when, sizeof(when), "20%02u-%02u-%02uT%02u:%02u:%02uZ",
             utc->year, utc->month, utc->day, utc->hours, utc->minutes,
             utc->seconds);
  }
  if (wp->flags & WAYPOINT_HAS_ALTITUDE) {
    snprintf(alt, sizeof(alt), "%.1f", (double)wp->altitude_m);
  }
  char line[160];
  int length = snprintf(line, sizeof(line),
                        "#WP,%u,%s,%s,%s,%s,%.1f,%.1f,%.1f,%08lx\n", wp->id,
                        when, lat, lon, alt, (double)wp->east_m,
                        (double)wp->north_m, (double)wp->track_m,
                        (unsigned long)wp->session);
  if (length <= 0 || (size_t)length >= sizeof(line)) {
    return;
  }
  printf("%s", line);
  if (time_valid) {
    sd_log_append(line, (size_t)length, utc, true,
                  datetime_to_seconds(utc, CENTURY_BASE));
  } else {
    ds3231_datetime_t epoch = {.day = 1, .month = 1, .year = 0};
    sd_log_append(line, (size_t)length, &epoch, false, 0);
  }
}

static void waypoint_mark(app_t *app) {
  if (!app->wps_ok) {
    show_error(app, "WP flash error");
    return;
  }
  if (waypoints_full(&app->wps)) {
    show_error(app, "WP memory full");
    return;
  }
  waypoint_t wp = {
      .session = app->session,
      .east_m = app->fusion.east_m,
      .north_m = app->fusion.north_m,
      .track_m = app->track_m,
  };
  if (fusion_lat_lon(&app->fusion, &wp.lat_e7, &wp.lon_e7)) {
    wp.flags |= WAYPOINT_HAS_LATLON;
  }
  if (app->alt.valid) {
    wp.altitude_m = alt_altitude_m(&app->alt);
    wp.flags |= WAYPOINT_HAS_ALTITUDE;
  }
  ds3231_datetime_t utc = {0};
  uint32_t now_s = 0;
  bool time_valid = rtc_now(app, &utc, &now_s);
  if (time_valid) {
    wp.utc_s = now_s;
    wp.flags |= WAYPOINT_HAS_TIME;
  }
  if (!waypoints_add(&app->wps, &wp)) {
    show_error(app, "WP write error");
    return;
  }
  app->wp_index = (uint16_t)(app->wps.count - 1u);
  waypoint_log(&wp, &utc, time_valid);
  char text[UI_COLUMNS + 1];
  snprintf(text, sizeof(text), "WP %u saved", wp.id);
  set_status(app, text, UI_COLOR_GREEN);
}

static void waypoints_erase(app_t *app) {
  if (!app->wps_ok) {
    return;
  }
  bool ok = waypoints_clear(&app->wps);
  app->wp_index = 0;
  printf("Waypoints cleared%s\n", ok ? "" : " (erase error)");
  set_status(app, ok ? "WP cleared" : "WP erase error",
             ok ? UI_COLOR_YELLOW : UI_COLOR_RED);
}

// ---------------------------------------------------------------------------
// WPT card
// ---------------------------------------------------------------------------

void draw_wpt_card(app_t *app) {
  cards_wpt_t c = {
      .storage_ok = app->wps_ok,
      .count = app->wps.count,
      .free_slots = app->wps_ok ? (uint16_t)(app->wps.storage.slot_count -
                                             app->wps.next_slot)
                                : 0,
      .selecting = app->select == SELECT_WPT,
  };
  uint16_t index = app->wp_index;
  if (c.selecting) {
    if (app->select_index >= app->wps.count) {
      c.clear_selected = true;
    }
    index = app->select_index;
  }
  if (index < app->wps.count && !c.clear_selected) {
    const waypoint_t *wp = &app->wps.items[index];
    c.has_item = true;
    c.id = wp->id;
    c.latlon_valid = (wp->flags & WAYPOINT_HAS_LATLON) != 0;
    c.lat_e7 = wp->lat_e7;
    c.lon_e7 = wp->lon_e7;
    c.altitude_valid = (wp->flags & WAYPOINT_HAS_ALTITUDE) != 0;
    c.altitude_m = wp->altitude_m;
    measure_point_t p;
    if (waypoint_local(app, wp, &p)) {
      measure_point_t here = nav_here(app);
      float de = p.east_m - here.east_m;
      float dn = p.north_m - here.north_m;
      c.relative_valid = true;
      c.distance_m = sqrtf(de * de + dn * dn);
      c.bearing_deg = geo_bearing_deg(de, dn);
      float heading;
      if (c.distance_m >= PDR_MIN_RANGE_M &&
          walking_heading(app, &heading, NULL)) {
        c.turn_valid = true;
        c.turn_deg = geo_angle_delta(c.bearing_deg, heading);
      }
    }
    if (c.altitude_valid && app->alt.valid) {
      c.dalt_valid = true;
      c.dalt_m = wp->altitude_m - alt_altitude_m(&app->alt);
    }
    if (wp->flags & WAYPOINT_HAS_TIME) {
      ds3231_datetime_t local;
      seconds_to_datetime((uint32_t)(wp->utc_s + local_offset_s(wp->utc_s)),
                          &local);
      c.time_valid = true;
      c.hour = local.hours;
      c.minute = local.minutes;
      c.day = local.day;
      c.month = local.month;
    }
  }
  ui_row_t rows[UI_CONTENT_ROWS];
  cards_wpt(&c, rows);
  ui_draw_rows(app->oled, rows);
}

void wpt_ok(app_t *app) {
  if (confirm_pending(app, CONFIRM_WPT_CLEAR)) {
    app->confirm = CONFIRM_NONE;
    waypoints_erase(app);
    return;
  }
  if (app->select == SELECT_WPT) {
    app->select = SELECT_NONE;
    if (app->select_index >= app->wps.count) {
      if (app->wps.count > 0) {
        confirm_request(app, CONFIRM_WPT_CLEAR, "OK: clear all?");
      }
      return;
    }
    app->wp_index = app->select_index;
    return;
  }
  waypoint_mark(app);
}
