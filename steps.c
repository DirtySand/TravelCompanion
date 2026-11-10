#include "steps.h"
#include <math.h>
#include <stddef.h>

#define SECONDS_PER_MINUTE 60.0f
#define CADENCE_ALPHA 0.3f

void steps_default_config(steps_config_t *config) {
  *config = (steps_config_t){
      .filter_alpha = 0.35f,
      .peak_threshold = 0.6f,
      .hysteresis = 0.4f,
      .min_interval_s = 0.3f,
      .max_interval_s = 2.0f,
      .confirm_steps = 4,
      .weinberg_k = 0.5f,
      .min_step_m = 0.25f,
      .max_step_m = 1.4f,
  };
}

void steps_init(steps_t *s, const steps_config_t *config) {
  *s = (steps_t){0};
  if (config != NULL) {
    s->config = *config;
  } else {
    steps_default_config(&s->config);
  }
  s->phase = STEPS_SEEK_PEAK;
}

static float step_length(const steps_config_t *c, float amplitude) {
  if (amplitude < 0.0f) {
    amplitude = 0.0f;
  }
  float length = c->weinberg_k * sqrtf(sqrtf(amplitude));
  if (length < c->min_step_m) {
    return c->min_step_m;
  }
  return (length > c->max_step_m) ? c->max_step_m : length;
}

static void end_series(steps_t *s) {
  s->walking = false;
  s->pending_steps = 0;
  s->pending_distance_m = 0.0f;
  s->cadence_spm = 0.0f;
}

static uint8_t register_step(steps_t *s, float *distance_m) {
  const steps_config_t *c = &s->config;
  float interval = s->peak_time_s - s->last_step_time_s;
  bool continues = s->have_step && interval <= c->max_interval_s;
  if (!continues) {
    end_series(s);
  } else {
    float cadence = SECONDS_PER_MINUTE / interval;
    s->cadence_spm =
        (s->cadence_spm > 0.0f)
            ? s->cadence_spm + CADENCE_ALPHA * (cadence - s->cadence_spm)
            : cadence;
  }
  float length = step_length(c, s->peak - s->valley);
  s->last_step_m = length;
  s->last_step_time_s = s->peak_time_s;
  s->have_step = true;

  if (s->walking) {
    *distance_m += length;
    return 1;
  }
  s->pending_steps++;
  s->pending_distance_m += length;
  if (s->pending_steps < c->confirm_steps) {
    return 0;
  }
  uint8_t committed = s->pending_steps;
  *distance_m += s->pending_distance_m;
  s->pending_steps = 0;
  s->pending_distance_m = 0.0f;
  s->walking = true;
  return committed;
}

uint8_t steps_update(steps_t *s, float vertical_accel_ms2, float dt_s,
                     float *distance_m) {
  const steps_config_t *c = &s->config;
  s->time_s += dt_s;
  if (!s->filter_ready) {
    s->stage1 = vertical_accel_ms2;
    s->stage2 = vertical_accel_ms2;
    s->peak = vertical_accel_ms2;
    s->peak_time_s = s->time_s;
    s->valley = vertical_accel_ms2;
    s->filter_ready = true;
  }
  s->stage1 += c->filter_alpha * (vertical_accel_ms2 - s->stage1);
  s->stage2 += c->filter_alpha * (s->stage1 - s->stage2);
  float y = s->stage2;

  if (s->have_step && s->time_s - s->last_step_time_s > c->max_interval_s &&
      (s->walking || s->pending_steps > 0)) {
    end_series(s);
  }

  uint8_t committed = 0;
  if (s->phase == STEPS_SEEK_PEAK) {
    if (y > s->peak) {
      s->peak = y;
      s->peak_time_s = s->time_s;
    }
    if (y < s->valley) {
      s->valley = y;
    }
    if (s->peak >= c->peak_threshold && y <= s->peak - c->hysteresis) {
      bool too_soon = s->have_step &&
                      s->peak_time_s - s->last_step_time_s < c->min_interval_s;
      if (!too_soon) {
        committed = register_step(s, distance_m);
      }
      s->phase = STEPS_SEEK_VALLEY;
      s->valley = y;
    }
  } else {
    if (y < s->valley) {
      s->valley = y;
    }
    if (y >= s->valley + c->hysteresis) {
      s->phase = STEPS_SEEK_PEAK;
      s->peak = y;
      s->peak_time_s = s->time_s;
    }
  }
  return committed;
}

bool steps_walking(const steps_t *s) {
  return s->walking &&
         s->time_s - s->last_step_time_s <= s->config.max_interval_s;
}
