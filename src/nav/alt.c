#include "alt.h"
#include <math.h>
#include <stddef.h>

#define SEA_LEVEL_PA 101325.0f
#define NWS_EXPONENT 0.190284f
#define NWS_SCALE_M 44307.69f

void alt_default_config(alt_config_t *config) {
  *config = (alt_config_t){
      .smoothing_s = 4.0f,
      .threshold_m = 3.0f,
      .offset_time_constant_s = 120.0f,
  };
}

void alt_init(alt_t *a, const alt_config_t *config) {
  *a = (alt_t){0};
  if (config != NULL) {
    a->config = *config;
  } else {
    alt_default_config(&a->config);
  }
}

float alt_pressure_to_m(float pressure_pa) {
  return NWS_SCALE_M * (1.0f - powf(pressure_pa / SEA_LEVEL_PA, NWS_EXPONENT));
}

static float ema_weight(float dt_s, float tau_s) {
  if (tau_s <= 0.0f || dt_s >= tau_s) {
    return 1.0f;
  }
  return (dt_s > 0.0f) ? dt_s / tau_s : 0.0f;
}

static void close_segment(alt_t *a, float dh) {
  float h = a->segment_horizontal_m;
  a->slope_extra_m += sqrtf(h * h + dh * dh) - h;
  a->segment_horizontal_m = 0.0f;
}

void alt_update(alt_t *a, float pressure_pa, float dt_s, bool tracking) {
  if (!(pressure_pa > 30000.0f && pressure_pa < 110000.0f)) {
    return;
  }
  float h = alt_pressure_to_m(pressure_pa);
  a->pressure_pa = pressure_pa;
  if (!a->valid) {
    a->baro_m = h;
    a->valid = true;
  } else {
    a->baro_m += ema_weight(dt_s, a->config.smoothing_s) * (h - a->baro_m);
  }
  if (!a->ref_valid) {
    a->ref_m = a->baro_m;
    a->ref_valid = true;
    return;
  }
  float dh = a->baro_m - a->ref_m;
  if (fabsf(dh) < a->config.threshold_m) {
    return;
  }
  if (tracking) {
    if (dh > 0.0f) {
      a->ascent_m += dh;
    } else {
      a->descent_m -= dh;
    }
    close_segment(a, dh);
  } else {
    a->segment_horizontal_m = 0.0f;
  }
  a->ref_m = a->baro_m;
}

void alt_add_horizontal(alt_t *a, float horizontal_m) {
  a->segment_horizontal_m += horizontal_m;
}

void alt_gps(alt_t *a, float gps_altitude_m, float dt_s) {
  if (!a->valid) {
    return;
  }
  float offset = gps_altitude_m - a->baro_m;
  if (!a->offset_valid) {
    a->offset_m = offset;
    a->offset_valid = true;
    return;
  }
  a->offset_m +=
      ema_weight(dt_s, a->config.offset_time_constant_s) * (offset - a->offset_m);
}

float alt_altitude_m(const alt_t *a) {
  return a->offset_valid ? a->baro_m + a->offset_m : a->baro_m;
}

void alt_reset_totals(alt_t *a) {
  a->ascent_m = 0.0f;
  a->descent_m = 0.0f;
  a->segment_horizontal_m = 0.0f;
  a->slope_extra_m = 0.0f;
  a->ref_valid = false;
}
