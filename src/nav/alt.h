#ifndef ALT_H
#define ALT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float smoothing_s;
  float threshold_m;
  float offset_time_constant_s;
} alt_config_t;

typedef struct {
  alt_config_t config;
  bool valid;
  float pressure_pa;
  float baro_m;
  bool offset_valid;
  float offset_m;

  bool ref_valid;
  float ref_m;
  float ascent_m;
  float descent_m;
  float segment_horizontal_m;
  float slope_extra_m;
} alt_t;

void alt_default_config(alt_config_t *config);
void alt_init(alt_t *a, const alt_config_t *config);

float alt_pressure_to_m(float pressure_pa);

void alt_update(alt_t *a, float pressure_pa, float dt_s, bool tracking);

void alt_add_horizontal(alt_t *a, float horizontal_m);

void alt_gps(alt_t *a, float gps_altitude_m, float dt_s);

float alt_altitude_m(const alt_t *a);

void alt_reset_totals(alt_t *a);

#endif // ALT_H
