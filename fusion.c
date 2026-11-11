#include "fusion.h"
#include <math.h>
#include <stddef.h>

void fusion_default_config(fusion_config_t *config) {
  *config = (fusion_config_t){
      .drift_fraction = 0.05f,
      .gps_sigma_min_m = 3.0f,
      .gps_sigma_per_hdop_m = 3.0f,
      .gate_sigmas = 4.0f,
      .max_rejects = 5,
      .sigma_floor_m = 2.0f,
      .idle_growth_m_per_s = 0.02f,
  };
}

void fusion_init(fusion_t *f, const fusion_config_t *config) {
  *f = (fusion_t){0};
  if (config != NULL) {
    f->config = *config;
  } else {
    fusion_default_config(&f->config);
  }
}

static float predicted_variance(const fusion_t *f) {
  float drift = f->config.drift_fraction * f->distance_since_m;
  float idle = f->config.idle_growth_m_per_s * f->idle_s;
  return f->variance_m2 + drift * drift + idle * idle;
}

void fusion_predict(fusion_t *f, float d_east_m, float d_north_m) {
  f->east_m += d_east_m;
  f->north_m += d_north_m;
  f->distance_since_m += sqrtf(d_east_m * d_east_m + d_north_m * d_north_m);
}

void fusion_step_unknown(fusion_t *f, float distance_m) {
  f->distance_since_m += distance_m / f->config.drift_fraction;
}

void fusion_idle(fusion_t *f, float dt_s) {
  if (dt_s > 0.0f) {
    f->idle_s += dt_s;
  }
}

static void restart_at(fusion_t *f, float east_m, float north_m, float r) {
  f->east_m = east_m;
  f->north_m = north_m;
  f->variance_m2 = r;
  f->distance_since_m = 0.0f;
  f->idle_s = 0.0f;
  f->rejects = 0;
}

bool fusion_update_gps(fusion_t *f, int32_t lat_e7, int32_t lon_e7,
                       float hdop, float moving_without_steps_m) {
  const fusion_config_t *c = &f->config;
  float sigma = c->gps_sigma_per_hdop_m * hdop;
  if (!(sigma >= c->gps_sigma_min_m)) {
    sigma = c->gps_sigma_min_m;
  }
  float r = sigma * sigma;

  if (!f->origin.valid) {
    geo_origin_t fix;
    geo_origin_set(&fix, lat_e7, lon_e7);
    int32_t origin_lat, origin_lon;
    geo_from_local(&fix, -f->east_m, -f->north_m, &origin_lat, &origin_lon);
    geo_origin_set(&f->origin, origin_lat, origin_lon);
    restart_at(f, f->east_m, f->north_m, r);
    f->updates++;
    return true;
  }

  float z_east, z_north;
  geo_to_local(&f->origin, lat_e7, lon_e7, &z_east, &z_north);
  float p = predicted_variance(f);
  if (moving_without_steps_m > 0.0f) {
    p += moving_without_steps_m * moving_without_steps_m;
  }
  float d_east = z_east - f->east_m;
  float d_north = z_north - f->north_m;
  float innovation2 = d_east * d_east + d_north * d_north;
  float gate = c->gate_sigmas * c->gate_sigmas * (p + r);
  if (innovation2 > gate) {
    f->rejected++;
    if (++f->rejects < c->max_rejects) {
      return false;
    }
    f->resets++;
    restart_at(f, z_east, z_north, r);
    f->updates++;
    return true;
  }
  float k = p / (p + r);
  f->east_m += k * d_east;
  f->north_m += k * d_north;
  float posterior = (1.0f - k) * p;
  float floor2 = c->sigma_floor_m * c->sigma_floor_m;
  f->variance_m2 = (posterior < floor2) ? floor2 : posterior;
  f->distance_since_m = 0.0f;
  f->idle_s = 0.0f;
  f->rejects = 0;
  f->updates++;
  return true;
}

fusion_source_t fusion_source(const fusion_t *f) {
  return f->origin.valid ? FUSION_SOURCE_GPS_PDR : FUSION_SOURCE_PDR;
}

float fusion_sigma_m(const fusion_t *f) {
  return sqrtf(predicted_variance(f));
}

bool fusion_lat_lon(const fusion_t *f, int32_t *lat_e7, int32_t *lon_e7) {
  if (!f->origin.valid) {
    return false;
  }
  geo_from_local(&f->origin, f->east_m, f->north_m, lat_e7, lon_e7);
  return true;
}
