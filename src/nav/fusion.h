#ifndef FUSION_H
#define FUSION_H

#include "geo.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float drift_fraction;
  float gps_sigma_min_m;
  float gps_sigma_per_hdop_m;
  float gate_sigmas;
  uint8_t max_rejects;
  float sigma_floor_m;
  float idle_growth_m_per_s;
} fusion_config_t;

typedef enum {
  FUSION_SOURCE_NONE,
  FUSION_SOURCE_PDR,
  FUSION_SOURCE_GPS_PDR,
} fusion_source_t;

typedef struct {
  fusion_config_t config;
  geo_origin_t origin;
  float east_m;
  float north_m;
  float variance_m2;
  float distance_since_m;
  float idle_s;
  uint8_t rejects;
  uint32_t updates;
  uint32_t rejected;
  uint32_t resets;
} fusion_t;

void fusion_default_config(fusion_config_t *config);
void fusion_init(fusion_t *f, const fusion_config_t *config);

void fusion_predict(fusion_t *f, float d_east_m, float d_north_m);

void fusion_step_unknown(fusion_t *f, float distance_m);

void fusion_idle(fusion_t *f, float dt_s);

bool fusion_update_gps(fusion_t *f, int32_t lat_e7, int32_t lon_e7,
                       float hdop, float moving_without_steps_m);

fusion_source_t fusion_source(const fusion_t *f);

float fusion_sigma_m(const fusion_t *f);

bool fusion_lat_lon(const fusion_t *f, int32_t *lat_e7, int32_t *lon_e7);

#endif // FUSION_H
