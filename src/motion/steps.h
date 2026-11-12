#ifndef STEPS_H
#define STEPS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float filter_alpha;
  float peak_threshold;
  float hysteresis;
  float min_interval_s;
  float max_interval_s;
  uint8_t confirm_steps;
  float weinberg_k;
  float min_step_m;
  float max_step_m;
} steps_config_t;

typedef enum {
  STEPS_SEEK_PEAK,
  STEPS_SEEK_VALLEY,
} steps_phase_t;

typedef enum {
  STEPS_PEAK_TOO_SOON,
  STEPS_PEAK_PENDING,
  STEPS_PEAK_CONFIRMED,
  STEPS_PEAK_WALKING,
} steps_peak_result_t;

typedef struct {
  uint32_t count;
  float time_s;
  float peak;
  float valley;
  float rise_s;
  float interval_s;
  uint8_t pending;
  steps_peak_result_t result;
} steps_peak_t;

typedef struct {
  steps_config_t config;
  bool filter_ready;
  float stage1;
  float stage2;
  float time_s;
  steps_phase_t phase;
  float peak;
  float peak_time_s;
  float valley;
  float valley_time_s;
  bool have_step;
  float last_step_time_s;
  uint8_t pending_steps;
  float pending_distance_m;
  bool walking;
  float last_step_m;
  float cadence_spm;
  steps_peak_t last_peak;
} steps_t;

void steps_default_config(steps_config_t *config);
void steps_init(steps_t *s, const steps_config_t *config);

uint8_t steps_update(steps_t *s, float vertical_accel_ms2, float dt_s,
                     float *distance_m);

bool steps_walking(const steps_t *s);

#endif // STEPS_H
