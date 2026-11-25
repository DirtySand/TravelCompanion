#include "app.h"
#include "board_config.h"
#include "datetime_utils.h"
#include "full_record.h"
#include "log.h"
#include "sd_log.h"
#include <stdio.h>
#include <string.h>

// Full log: while active (manual mode only), one #F record every
// FULL_LOG_PERIOD_MS with everything the device knows, plus (FULL_LOG_RAW_IMU)
// every IMU FIFO frame as an #I line. GPS runs continuously instead of in
// Power Save Mode. Everything goes into the normal encrypted SD log.

#define FULL_LOG_CLIMATE_MAX_AGE_MS 900
#define FULL_LOG_PRESSURE_MAX_AGE_MS 1500

static uint32_t t_ms(uint64_t us) { return (uint32_t)(us / 1000u); }

static void read_time(app_t *app) {
  full_log_t *f = &app->full_log;
  ds3231_datetime_t utc;
  if (ds3231_read_datetime(app->ds3231, &utc) != DS3231_OK) {
    f->time_valid = false;
    return;
  }
  f->utc = utc;
  f->utc_s = datetime_to_seconds(&utc, CENTURY_BASE);
  f->time_valid = !app->time_invalid;
}

static void append(app_t *app, const char *text, size_t length) {
  full_log_t *f = &app->full_log;
  if (length == 0) {
    return;
  }
  sd_log_append(text, length, &f->utc, f->time_valid, f->utc_s);
  f->bytes += length;
}

static void emit_chunk(app_t *app) {
  full_log_t *f = &app->full_log;
  append(app, f->chunk, f->chunk_length);
  f->chunk_length = 0;
}

static void write_meta(app_t *app) {
  const full_log_t *f = &app->full_log;
  full_meta_t m = {
      .utc = f->utc,
      .time_valid = f->time_valid,
      .t_ms = t_ms(f->started_us),
      .session = app->session,
      .period_ms = FULL_LOG_PERIOD_MS,
      .imu_period_us = imu_sample_period_us(),
      .imu_axes = {IMU_AXIS_FORWARD, IMU_AXIS_RIGHT, IMU_AXIS_DOWN},
      .mag_axes = {MAG_AXIS_FORWARD, MAG_AXIS_RIGHT, MAG_AXIS_DOWN},
      .declination_deg = MAG_DECLINATION_DEG,
  };
  if (app->imu_ok) {
    m.accel_lsb_per_g = icm20649_accel_lsb_per_g(app->imu);
    m.gyro_lsb_per_dps = icm20649_gyro_lsb_per_dps(app->imu);
  }
  char line[FULL_META_CSV_MAX];
  append(app, line, full_meta_to_csv(&m, line, sizeof(line)));
}

static void reset_period(full_log_t *f) {
  f->period_frames = 0;
  f->accel_sum_g = (compass_vec_t){0};
  f->gyro_sum_dps = (compass_vec_t){0};
}

static void full_log_start(app_t *app) {
  if (sd_log_state() != SD_LOG_READY) {
    set_status(app, "SD not ready", UI_COLOR_RED);
    return;
  }
  full_log_t *f = &app->full_log;
  memset(f, 0, sizeof(*f));
  f->active = true;
  f->started_us = time_us_64();
  f->next_at = get_absolute_time();
  read_time(app);
  write_meta(app);
  if (app->gps_enabled) {
    gps_request(app, GPS_CMD_POWER_MODE);
  }
  LOG("Full log started\n");
  set_status(app, "Full log ON", UI_COLOR_GREEN);
}

static void full_log_stop(app_t *app) {
  full_log_t *f = &app->full_log;
  read_time(app);
  emit_chunk(app);
  sd_log_flush(f->utc_s);
  f->active = false;
  if (app->gps_enabled) {
    gps_request(app, GPS_CMD_POWER_MODE);
  }
  LOG("Full log stopped: %lu records, %lu IMU frames\n",
      (unsigned long)f->records, (unsigned long)f->imu_frames);
  set_status(app, "Full log OFF", UI_COLOR_GRAY);
}

void full_log_toggle(app_t *app) {
  if (app->full_log.active) {
    full_log_stop(app);
  } else {
    full_log_start(app);
  }
}

void full_log_imu_frame(app_t *app, const int16_t raw[6], uint64_t frame_us,
                        const compass_vec_t *accel_g,
                        const compass_vec_t *gyro_dps) {
  full_log_t *f = &app->full_log;
  if (!f->active) {
    return;
  }
  f->imu_frames++;
  f->period_frames++;
  f->accel_sum_g.x += accel_g->x;
  f->accel_sum_g.y += accel_g->y;
  f->accel_sum_g.z += accel_g->z;
  f->gyro_sum_dps.x += gyro_dps->x;
  f->gyro_sum_dps.y += gyro_dps->y;
  f->gyro_sum_dps.z += gyro_dps->z;
  if (!FULL_LOG_RAW_IMU) {
    return;
  }
  char line[FULL_IMU_CSV_MAX];
  size_t length = full_imu_to_csv(t_ms(frame_us), raw, line, sizeof(line));
  if (f->chunk_length + length > sizeof(f->chunk)) {
    emit_chunk(app);
  }
  memcpy(&f->chunk[f->chunk_length], line, length);
  f->chunk_length += length;
}

void full_log_imu_overflow(app_t *app) {
  if (app->full_log.active) {
    app->full_log.imu_overflows++;
  }
}

// ---------------------------------------------------------------------------
// #F record
// ---------------------------------------------------------------------------

static void collect_gps_fields(app_t *app, full_record_t *r) {
  if (!app->gps_enabled) {
    r->gps = SAMPLE_GPS_OFF;
    return;
  }
  if (!gps_is_fresh(app)) {
    r->gps = SAMPLE_GPS_NO_DATA;
    return;
  }
  neo6m_data_t live;
  neo6m_get_data(app->gps, &live);
  r->fix_quality = live.fix_quality;
  r->satellites = live.satellites;
  r->sky_valid = live.sky_valid;
  r->satellites_in_view = live.satellites_in_view;
  r->cno_top4_avg = live.cno_top4_avg;
  if (!live.fix_valid || !live.position_valid) {
    r->gps = SAMPLE_GPS_NO_FIX;
    return;
  }
  r->gps = SAMPLE_GPS_FIX;
  r->latitude_e7 = live.latitude_e7;
  r->longitude_e7 = live.longitude_e7;
  r->altitude_valid = live.altitude_valid;
  r->altitude_dm = live.altitude_dm;
  r->speed_valid = live.speed_valid;
  r->speed_kmh_x10 = live.speed_kmh_x10;
  r->course_valid = live.course_valid;
  r->course_x100 = live.course_x100;
  r->hdop_valid = live.hdop_valid;
  r->hdop_x100 = live.hdop_x100;
}

static void collect_attitude(app_t *app, full_record_t *r) {
  bool imu = app->imu_ok && !app->imu_error;
  bool mag = app->mag_ok && !app->mag_error && app->mag_filter_valid;
  if (mag) {
    r->mag_valid = true;
    r->mag_ut[0] = app->mag_field.x;
    r->mag_ut[1] = app->mag_field.y;
    r->mag_ut[2] = app->mag_field.z;
  }
  if (!imu) {
    return;
  }
  attitude_output_t att;
  attitude_output(&app->attitude, MAG_DECLINATION_DEG,
                  COMPASS_MIN_HORIZONTAL_UT, &att);
  r->tilt_valid = app->attitude.down_valid;
  r->pitch_deg = att.pitch_deg;
  r->roll_deg = att.roll_deg;
  r->field_valid = mag && att.field_valid;
  r->field_ut = att.field_ut;
  r->inclination_deg = att.inclination_deg;
  r->heading_valid = r->field_valid && att.heading_valid;
  r->heading_true_deg = att.heading_true_deg;
  r->heading_magnetic_deg = att.heading_magnetic_deg;
  r->heading_ref =
      (att.axis == ATTITUDE_AXIS_SCREEN) ? FULL_HDG_REF_BACK : FULL_HDG_REF_TOP;
}

static void collect_imu_fields(app_t *app, full_record_t *r) {
  const full_log_t *f = &app->full_log;
  r->imu_frames = f->period_frames;
  r->imu_overflows = f->imu_overflows;
  if (f->period_frames > 0) {
    float n = (float)f->period_frames;
    r->accel_g[0] = f->accel_sum_g.x / n;
    r->accel_g[1] = f->accel_sum_g.y / n;
    r->accel_g[2] = f->accel_sum_g.z / n;
    r->gyro_dps[0] = f->gyro_sum_dps.x / n;
    r->gyro_dps[1] = f->gyro_sum_dps.y / n;
    r->gyro_dps[2] = f->gyro_sum_dps.z / n;
  }
  icm20649_raw_t raw;
  if (app->imu_ok && icm20649_read_raw(app->imu, &raw) == ICM20649_OK) {
    r->imu_temp_valid = true;
    r->imu_temp_c = icm20649_temperature_c(raw.temperature);
  }
}

static void collect_motion(app_t *app, full_record_t *r) {
  r->steps = app->steps_total;
  r->cadence_spm = app->steps.cadence_spm;
  r->walking = steps_walking(&app->steps);
  r->track_m = app->track_m;
  switch (app->trip.state) {
  case TRIP_RUNNING:
    r->trip = FULL_TRIP_RUNNING;
    break;
  case TRIP_PAUSED:
    r->trip = FULL_TRIP_PAUSED;
    break;
  default:
    r->trip = FULL_TRIP_STOPPED;
    break;
  }
  r->trip_steps = app->trip.steps;
  r->trip_m = app->trip.distance_m;

  switch (fusion_source(&app->fusion)) {
  case FUSION_SOURCE_GPS_PDR:
    r->pos_source = FULL_POS_GPS_PDR;
    break;
  case FUSION_SOURCE_PDR:
    r->pos_source = FULL_POS_PDR;
    break;
  default:
    r->pos_source = FULL_POS_NONE;
    break;
  }
  r->east_m = app->fusion.east_m;
  r->north_m = app->fusion.north_m;
  r->sigma_m = fusion_sigma_m(&app->fusion);
  r->pos_latlon_valid =
      fusion_lat_lon(&app->fusion, &r->pos_lat_e7, &r->pos_lon_e7);
  r->dr_east_m = app->dr_east_m;
  r->dr_north_m = app->dr_north_m;
}

static void write_record(app_t *app) {
  full_log_t *f = &app->full_log;
  full_record_t r = {
      .utc = f->utc,
      .time_valid = f->time_valid,
      .t_ms = t_ms(time_us_64()),
  };
  r.climate_valid = env_climate(app, FULL_LOG_CLIMATE_MAX_AGE_MS,
                                &r.temperature_c, &r.humidity_pct);
  r.pressure_valid =
      env_pressure(app, FULL_LOG_PRESSURE_MAX_AGE_MS, &r.pressure_pa);
  collect_gps_fields(app, &r);
  r.baro_valid = app->alt.valid;
  r.baro_altitude_m = alt_altitude_m(&app->alt);
  r.baro_gps_ref = app->alt.offset_valid;
  r.ascent_m = app->alt.ascent_m;
  r.descent_m = app->alt.descent_m;
  collect_attitude(app, &r);
  collect_imu_fields(app, &r);
  collect_motion(app, &r);

  char line[FULL_RECORD_CSV_MAX];
  size_t length = full_record_to_csv(&r, line, sizeof(line));
  if (length == 0) {
    LOG("Full log record too long\n");
    return;
  }
  append(app, line, length);
  f->records++;
}

void full_log_service(app_t *app) {
  full_log_t *f = &app->full_log;
  if (!f->active || !time_reached(f->next_at)) {
    return;
  }
  f->next_at = delayed_by_ms(f->next_at, FULL_LOG_PERIOD_MS);
  if (time_reached(f->next_at)) {
    f->next_at = make_timeout_time_ms(FULL_LOG_PERIOD_MS);
  }
  read_time(app);
  emit_chunk(app);
  write_record(app);
  reset_period(f);
}

absolute_time_t full_log_wake_at(const app_t *app, absolute_time_t wake_at) {
  if (app->full_log.active &&
      absolute_time_diff_us(app->full_log.next_at, wake_at) > 0) {
    return app->full_log.next_at;
  }
  return wake_at;
}

// ---------------------------------------------------------------------------
// LOG card
// ---------------------------------------------------------------------------

static cards_sd_t sd_state(void) {
  switch (sd_log_state()) {
  case SD_LOG_READY:
    return CARDS_SD_READY;
  case SD_LOG_ERROR:
    return CARDS_SD_ERROR;
  default:
    return CARDS_SD_NO_CARD;
  }
}

void draw_log_card(app_t *app) {
  const full_log_t *f = &app->full_log;
  cards_log_t c = {
      .active = f->active,
      .period_ms = FULL_LOG_PERIOD_MS,
      .records = f->records,
      .raw_imu = FULL_LOG_RAW_IMU != 0,
      .imu_frames = f->imu_frames,
      .imu_overflows = f->imu_overflows,
      .bytes = f->bytes,
      .sd = sd_state(),
      .gps_on = app->gps_enabled,
  };
  if (f->active) {
    c.elapsed_s = (uint32_t)((time_us_64() - f->started_us) / 1000000u);
  }
  ui_row_t rows[UI_CONTENT_ROWS];
  cards_log(&c, rows);
  ui_draw_rows(app->oled, rows);
}
