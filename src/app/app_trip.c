#include "app.h"

ui_trip_state_t ui_trip_state(const trip_t *t) {
  if (t->state == TRIP_RUNNING) {
    return UI_TRIP_RUNNING;
  }
  return (t->state == TRIP_PAUSED) ? UI_TRIP_PAUSED : UI_TRIP_STOPPED;
}

void draw_trip_card(app_t *app) {
  uint64_t now_us = time_us_64();
  const trip_t *t = &app->trip;
  bool walking = steps_walking(&app->steps);
  float speed_ms =
      walking ? app->steps.cadence_spm / 60.0f * app->steps.last_step_m : 0.0f;
  ui_trip_t trip = {
      .sensor_ok = app->imu_ok && !app->imu_error && app->imu_fifo_running,
      .state = ui_trip_state(t),
      .elapsed_s = (uint32_t)(trip_elapsed_us(t, now_us) / 1000000u),
      .steps = t->steps,
      .distance_m = t->distance_m,
      .walking = walking,
      .speed_kmh = speed_ms * MS_TO_KMH,
      .avg_speed_kmh = trip_average_speed(t, now_us) * MS_TO_KMH,
      .cadence_spm = app->steps.cadence_spm,
      .last_step_m = app->steps.last_step_m,
      .weinberg_k = app->steps.config.weinberg_k,
  };
  ui_draw_trip_card(app->oled, &trip);
}

void draw_pdr_card(app_t *app) {
  const pdr_t *p = &app->pdr;
  ui_pdr_t pdr = {
      .sensor_ok = app->imu_ok && !app->imu_error && app->imu_fifo_running,
      .state = ui_trip_state(&app->trip),
      .east_m = p->east_m,
      .north_m = p->north_m,
      .range_m = pdr_range_m(p),
      .reference = UI_COMPASS_REF_BACK,
      .path_m = app->trip.distance_m,
      .steps_no_heading = p->steps_no_heading,
      .mag_calibrated = app->mag_cal.valid,
  };
  pdr.bearing_valid = pdr_bearing_deg(p, PDR_MIN_RANGE_M, &pdr.bearing_deg);
  pdr.heading_valid = walking_heading(app, &pdr.heading_deg, &pdr.reference);
  ui_draw_pdr_card(app->oled, &pdr);
}

void trip_ok(app_t *app) {
  trip_state_t state = trip_toggle(&app->trip, time_us_64());
  if (state == TRIP_RUNNING) {
    set_status(app, "Trip started", UI_COLOR_GREEN);
  } else {
    set_status(app, "Trip paused", UI_COLOR_YELLOW);
  }
}

void trip_long_ok(app_t *app) {
  if (trip_running(&app->trip)) {
    set_status(app, "Pause first", UI_COLOR_RED);
    return;
  }
  trip_reset(&app->trip);
  pdr_reset(&app->pdr);
  alt_reset_totals(&app->alt);
  set_status(app, "Trip reset", UI_COLOR_GRAY);
}
