#ifndef COMPASS_H
#define COMPASS_H

#include <stdbool.h>
#include <stdint.h>

#define COMPASS_AXIS_COUNT 3u
#define COMPASS_AXIS_BIT(axis) (1u << (axis))

typedef struct {
  float x;
  float y;
  float z;
} compass_vec_t;

typedef struct {
  int8_t forward;
  int8_t right;
  int8_t down;
} compass_axes_t;

typedef struct {
  bool valid;
  uint8_t axis_mask;
  compass_vec_t offset;
  compass_vec_t scale;
} compass_cal_t;

typedef struct {
  compass_vec_t min;
  compass_vec_t max;
  uint32_t samples;
} compass_cal_session_t;

typedef struct {
  bool heading_valid;
  float heading_magnetic_deg;
  float heading_true_deg;
  float field_ut;
  float horizontal_ut;
  float inclination_deg;
} compass_reading_t;

bool compass_axes_valid(const compass_axes_t *axes);
uint8_t compass_axis_index(int8_t code);

void compass_cal_begin(compass_cal_session_t *session);

void compass_cal_add(compass_cal_session_t *session, const compass_vec_t *raw);

void compass_cal_ranges(const compass_cal_session_t *session,
                        compass_vec_t *ranges);

bool compass_cal_finish(const compass_cal_session_t *session,
                        const compass_axes_t *axes, float min_range_ut,
                        compass_cal_t *out);

void compass_apply_cal(const compass_cal_t *cal, const compass_vec_t *raw,
                       compass_vec_t *out);

void compass_filter(compass_vec_t *state, const compass_vec_t *sample,
                    float alpha);

void compass_compute(const compass_axes_t *axes, const compass_vec_t *field,
                     float declination_deg, float min_horizontal_ut,
                     compass_reading_t *reading);

float compass_wrap_deg(float degrees);

#endif // COMPASS_H
