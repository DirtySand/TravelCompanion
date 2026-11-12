#include "compass.h"
#include <math.h>
#include <stddef.h>

#define RAD_TO_DEG (180.0f / 3.14159265358979f)

static float *component(compass_vec_t *v, uint8_t axis) {
  switch (axis) {
  case 0:
    return &v->x;
  case 1:
    return &v->y;
  default:
    return &v->z;
  }
}

static float get_component(const compass_vec_t *v, uint8_t axis) {
  switch (axis) {
  case 0:
    return v->x;
  case 1:
    return v->y;
  default:
    return v->z;
  }
}

uint8_t compass_axis_index(int8_t code) {
  int magnitude = (code < 0) ? -code : code;
  return (uint8_t)(magnitude - 1);
}

static float device_component(const compass_vec_t *v, int8_t code) {
  float value = get_component(v, compass_axis_index(code));
  return (code < 0) ? -value : value;
}

static bool code_in_range(int8_t code) {
  return code != 0 && code >= -3 && code <= 3;
}

bool compass_axes_valid(const compass_axes_t *axes) {
  if (axes == NULL || !code_in_range(axes->forward) ||
      !code_in_range(axes->right) || !code_in_range(axes->down)) {
    return false;
  }
  uint8_t used = (uint8_t)(COMPASS_AXIS_BIT(compass_axis_index(axes->forward)) |
                           COMPASS_AXIS_BIT(compass_axis_index(axes->right)) |
                           COMPASS_AXIS_BIT(compass_axis_index(axes->down)));
  return used == 0x07u;
}

static compass_vec_t axis_vector(int8_t code) {
  compass_vec_t v = {0};
  *component(&v, compass_axis_index(code)) = (code < 0) ? -1.0f : 1.0f;
  return v;
}

bool compass_axes_right_handed(const compass_axes_t *axes) {
  if (!compass_axes_valid(axes)) {
    return false;
  }
  compass_vec_t f = axis_vector(axes->forward);
  compass_vec_t r = axis_vector(axes->right);
  compass_vec_t d = axis_vector(axes->down);
  compass_vec_t cross = {
      .x = f.y * r.z - f.z * r.y,
      .y = f.z * r.x - f.x * r.z,
      .z = f.x * r.y - f.y * r.x,
  };
  return cross.x == d.x && cross.y == d.y && cross.z == d.z;
}

void compass_to_device(const compass_axes_t *axes, const compass_vec_t *sensor,
                       compass_vec_t *device) {
  compass_vec_t in = *sensor;
  device->x = device_component(&in, axes->forward);
  device->y = device_component(&in, axes->right);
  device->z = device_component(&in, axes->down);
}

void compass_tilt(const compass_vec_t *accel_device, float *pitch_deg,
                  float *roll_deg) {
  float f_x = accel_device->x;
  float f_y = accel_device->y;
  float f_z = accel_device->z;
  *pitch_deg = atan2f(f_x, sqrtf(f_y * f_y + f_z * f_z)) * RAD_TO_DEG;
  *roll_deg = atan2f(-f_y, -f_z) * RAD_TO_DEG;
}

void compass_cal_begin(compass_cal_session_t *session) {
  session->min = (compass_vec_t){0};
  session->max = (compass_vec_t){0};
  session->samples = 0;
}

void compass_cal_add(compass_cal_session_t *session, const compass_vec_t *raw) {
  if (session->samples == 0) {
    session->min = *raw;
    session->max = *raw;
  } else {
    for (uint8_t axis = 0; axis < COMPASS_AXIS_COUNT; axis++) {
      float value = get_component(raw, axis);
      if (value < get_component(&session->min, axis)) {
        *component(&session->min, axis) = value;
      }
      if (value > get_component(&session->max, axis)) {
        *component(&session->max, axis) = value;
      }
    }
  }
  session->samples++;
}

void compass_cal_ranges(const compass_cal_session_t *session,
                        compass_vec_t *ranges) {
  ranges->x = session->max.x - session->min.x;
  ranges->y = session->max.y - session->min.y;
  ranges->z = session->max.z - session->min.z;
}

bool compass_cal_finish(const compass_cal_session_t *session,
                        const compass_axes_t *axes, float min_range_ut,
                        compass_cal_t *out) {
  if (session->samples == 0 || !compass_axes_valid(axes)) {
    return false;
  }
  compass_vec_t ranges;
  compass_cal_ranges(session, &ranges);

  uint8_t mask = 0;
  for (uint8_t axis = 0; axis < COMPASS_AXIS_COUNT; axis++) {
    if (get_component(&ranges, axis) >= min_range_ut) {
      mask |= (uint8_t)COMPASS_AXIS_BIT(axis);
    }
  }
  uint8_t forward = compass_axis_index(axes->forward);
  uint8_t right = compass_axis_index(axes->right);
  uint8_t needed =
      (uint8_t)(COMPASS_AXIS_BIT(forward) | COMPASS_AXIS_BIT(right));
  if ((mask & needed) != needed) {
    return false;
  }

  float mean_range =
      0.5f * (get_component(&ranges, forward) + get_component(&ranges, right));
  compass_cal_t cal = {.valid = true, .axis_mask = mask};
  for (uint8_t axis = 0; axis < COMPASS_AXIS_COUNT; axis++) {
    bool calibrated = (mask & COMPASS_AXIS_BIT(axis)) != 0;
    *component(&cal.offset, axis) =
        calibrated ? 0.5f * (get_component(&session->min, axis) +
                             get_component(&session->max, axis))
                   : 0.0f;
    *component(&cal.scale, axis) =
        (axis == forward || axis == right)
            ? mean_range / get_component(&ranges, axis)
            : 1.0f;
  }
  *out = cal;
  return true;
}

void compass_apply_cal(const compass_cal_t *cal, const compass_vec_t *raw,
                       compass_vec_t *out) {
  if (cal == NULL || !cal->valid) {
    *out = *raw;
    return;
  }
  out->x = (raw->x - cal->offset.x) * cal->scale.x;
  out->y = (raw->y - cal->offset.y) * cal->scale.y;
  out->z = (raw->z - cal->offset.z) * cal->scale.z;
}

void compass_filter(compass_vec_t *state, const compass_vec_t *sample,
                    float alpha) {
  state->x += alpha * (sample->x - state->x);
  state->y += alpha * (sample->y - state->y);
  state->z += alpha * (sample->z - state->z);
}

float compass_wrap_deg(float degrees) {
  float wrapped = fmodf(degrees, 360.0f);
  if (wrapped < 0.0f) {
    wrapped += 360.0f;
  }
  if (wrapped >= 360.0f) {
    wrapped -= 360.0f;
  }
  return wrapped;
}

void compass_compute(const compass_axes_t *axes, const compass_vec_t *field,
                     float declination_deg, float min_horizontal_ut,
                     compass_reading_t *reading) {
  float forward = device_component(field, axes->forward);
  float right = device_component(field, axes->right);
  float down = device_component(field, axes->down);

  reading->horizontal_ut = sqrtf(forward * forward + right * right);
  reading->field_ut = sqrtf(forward * forward + right * right + down * down);
  reading->inclination_deg = atan2f(down, reading->horizontal_ut) * RAD_TO_DEG;
  reading->heading_valid = reading->horizontal_ut >= min_horizontal_ut;
  float magnetic = atan2f(-right, forward) * RAD_TO_DEG;
  reading->heading_magnetic_deg = compass_wrap_deg(magnetic);
  reading->heading_true_deg = compass_wrap_deg(magnetic + declination_deg);
}
