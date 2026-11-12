#include "app.h"
#include <math.h>

#define NAV_FIX_MAX_DT_S 10.0f
#define NAV_STEPLESS_MIN_KMH 3.0f
#define ALT_GPS_MAX_HDOP 2.0f
#define ALT_GPS_MIN_SATELLITES 6
#define BARO_PERIOD_MS 1000
#define BARO_RETRY_MS 10000

bool nav_heading_ok(const app_t *app) {
  return app->imu_ok && !app->imu_error && app->imu_fifo_running &&
         app->mag_ok && !app->mag_error;
}

void nav_steps(app_t *app, float distance_m, bool heading_valid,
                      float heading_deg) {
  app->track_m += distance_m;
  alt_add_horizontal(&app->alt, distance_m);
  app->steps_since_fix = true;
  if (!heading_valid) {
    fusion_step_unknown(&app->fusion, distance_m);
    app->dr_steps_no_heading = true;
    return;
  }
  float heading_rad = heading_deg * (float)(M_PI / 180.0);
  float d_east = distance_m * sinf(heading_rad);
  float d_north = distance_m * cosf(heading_rad);
  fusion_predict(&app->fusion, d_east, d_north);
  app->dr_east_m += d_east;
  app->dr_north_m += d_north;
}

void nav_gps(app_t *app) {
  if (app->mode != MODE_MANUAL) {
    return;
  }
  neo6m_data_t data;
  neo6m_get_data(app->gps, &data);
  if (!data.fix_valid || !data.position_valid) {
    return;
  }
  float dt_s = 1.0f;
  if (app->nav_fix_seen) {
    dt_s = (float)ms_since(app->nav_fix_at) / 1000.0f;
    if (dt_s > NAV_FIX_MAX_DT_S) {
      dt_s = NAV_FIX_MAX_DT_S;
    }
  }
  app->nav_fix_at = get_absolute_time();
  app->nav_fix_seen = true;

  float moving_m = 0.0f;
  bool steps_possible = app->imu_ok && !app->imu_error && app->imu_fifo_running;
  if (data.speed_valid && (!steps_possible || !app->steps_since_fix)) {
    float speed_kmh = (float)data.speed_kmh_x10 / 10.0f;
    if (!steps_possible || speed_kmh >= NAV_STEPLESS_MIN_KMH) {
      moving_m = speed_kmh / MS_TO_KMH * dt_s;
    }
  }
  float hdop = data.hdop_valid ? (float)data.hdop_x100 / 100.0f : 99.0f;
  fusion_update_gps(&app->fusion, data.latitude_e7, data.longitude_e7, hdop,
                    moving_m);
  app->steps_since_fix = false;

  if (data.altitude_valid && data.hdop_valid && hdop <= ALT_GPS_MAX_HDOP &&
      data.satellites >= ALT_GPS_MIN_SATELLITES) {
    alt_gps(&app->alt, (float)data.altitude_dm / 10.0f, dt_s);
  }
}

static void nav_baro(app_t *app) {
  if (!time_reached(app->baro_at)) {
    return;
  }
  absolute_time_t now = get_absolute_time();
  float dt_s = app->baro_seen
                   ? (float)absolute_time_diff_us(app->baro_last, now) / 1e6f
                   : 1.0f;
  app->baro_last = now;
  app->baro_seen = true;
  app->baro_at = make_timeout_time_ms(BARO_PERIOD_MS);
  float pressure_pa;
  if (read_pressure(app->bme680, &pressure_pa)) {
    alt_update(&app->alt, pressure_pa, dt_s, trip_running(&app->trip));
  } else {
    app->baro_at = make_timeout_time_ms(BARO_RETRY_MS);
  }
}

static void nav_idle(app_t *app) {
  absolute_time_t now = get_absolute_time();
  if (app->nav_idle_seen) {
    fusion_idle(&app->fusion,
                (float)absolute_time_diff_us(app->nav_idle_at, now) / 1e6f);
  }
  app->nav_idle_at = now;
  app->nav_idle_seen = true;
}

measure_point_t nav_here(const app_t *app) {
  return (measure_point_t){app->fusion.east_m, app->fusion.north_m};
}

measure_point_t nav_dr_here(const app_t *app) {
  return (measure_point_t){app->dr_east_m, app->dr_north_m};
}

cards_source_t nav_source(const app_t *app) {
  if (fusion_source(&app->fusion) == FUSION_SOURCE_GPS_PDR) {
    return nav_heading_ok(app) ? CARDS_SOURCE_GPS_PDR : CARDS_SOURCE_GPS;
  }
  return nav_heading_ok(app) ? CARDS_SOURCE_PDR : CARDS_SOURCE_NONE;
}

void nav_update(app_t *app) {
  imu_drain_fifo(app);
  mag_update(app);
  nav_idle(app);
  nav_baro(app);
}
