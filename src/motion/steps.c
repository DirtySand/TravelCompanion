#include "steps.h"
#include <math.h>
#include <stddef.h>

#define SECONDS_PER_MINUTE 60.0f
#define US_PER_S 1000000.0f
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

static float seconds_between(uint64_t later_us, uint64_t earlier_us) {
  return (float)(later_us - earlier_us) / US_PER_S;
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

static void note_peak(steps_t *s, steps_peak_result_t result) {
  steps_peak_t *p = &s->last_peak;
  p->count++;
  p->time_s = (float)s->peak_time_us / US_PER_S;
  p->peak = s->peak;
  p->valley = s->valley;
  p->rise_s = (s->peak_time_us >= s->valley_time_us)
                  ? seconds_between(s->peak_time_us, s->valley_time_us)
                  : 0.0f;
  p->pending = s->pending_steps;
  p->result = result;
}

static uint8_t register_step(steps_t *s, float *distance_m) {
  const steps_config_t *c = &s->config;
  float interval = seconds_between(s->peak_time_us, s->last_step_time_us);
  s->last_peak.interval_s = s->have_step ? interval : 0.0f;
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
  s->last_step_time_us = s->peak_time_us;
  s->have_step = true;

  if (s->walking) {
    *distance_m += length;
    note_peak(s, STEPS_PEAK_WALKING);
    return 1;
  }
  s->pending_steps++;
  s->pending_distance_m += length;
  if (s->pending_steps < c->confirm_steps) {
    note_peak(s, STEPS_PEAK_PENDING);
    return 0;
  }
  uint8_t committed = s->pending_steps;
  *distance_m += s->pending_distance_m;
  s->pending_steps = 0;
  s->pending_distance_m = 0.0f;
  s->walking = true;
  note_peak(s, STEPS_PEAK_CONFIRMED);
  return committed;
}

uint8_t steps_update(steps_t *s, float vertical_accel_ms2, float dt_s,
                     float *distance_m) {
  const steps_config_t *c = &s->config;
  s->time_us += (uint64_t)llroundf(dt_s * US_PER_S);
  if (!s->filter_ready) {
    s->stage1 = vertical_accel_ms2;
    s->stage2 = vertical_accel_ms2;
    s->peak = vertical_accel_ms2;
    s->peak_time_us = s->time_us;
    s->valley = vertical_accel_ms2;
    s->valley_time_us = s->time_us;
    s->filter_ready = true;
  }
  s->stage1 += c->filter_alpha * (vertical_accel_ms2 - s->stage1);
  s->stage2 += c->filter_alpha * (s->stage1 - s->stage2);
  float y = s->stage2;

  if (s->have_step &&
      seconds_between(s->time_us, s->last_step_time_us) > c->max_interval_s &&
      (s->walking || s->pending_steps > 0)) {
    end_series(s);
  }

  uint8_t committed = 0;
  if (s->phase == STEPS_SEEK_PEAK) {
    if (y > s->peak) {
      s->peak = y;
      s->peak_time_us = s->time_us;
    }
    if (y < s->valley) {
      s->valley = y;
      s->valley_time_us = s->time_us;
    }
    if (s->peak >= c->peak_threshold && y <= s->peak - c->hysteresis) {
      float since_step = seconds_between(s->peak_time_us, s->last_step_time_us);
      bool too_soon = s->have_step && since_step < c->min_interval_s;
      if (!too_soon) {
        committed = register_step(s, distance_m);
      } else {
        s->last_peak.interval_s = since_step;
        note_peak(s, STEPS_PEAK_TOO_SOON);
      }
      s->phase = STEPS_SEEK_VALLEY;
      s->valley = y;
      s->valley_time_us = s->time_us;
    }
  } else {
    if (y < s->valley) {
      s->valley = y;
      s->valley_time_us = s->time_us;
    }
    if (y >= s->valley + c->hysteresis) {
      s->phase = STEPS_SEEK_PEAK;
      s->peak = y;
      s->peak_time_us = s->time_us;
    }
  }
  return committed;
}

bool steps_walking(const steps_t *s) {
  return s->walking && seconds_between(s->time_us, s->last_step_time_us) <=
                           s->config.max_interval_s;
}
