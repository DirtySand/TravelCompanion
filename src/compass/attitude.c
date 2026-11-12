#include "attitude.h"
#include <math.h>
#include <stddef.h>

#define DEG_TO_RAD (3.14159265358979f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.14159265358979f)
#define MIN_ACCEL_G 0.5f

static float dot(const compass_vec_t *a, const compass_vec_t *b) {
  return a->x * b->x + a->y * b->y + a->z * b->z;
}

static compass_vec_t cross(const compass_vec_t *a, const compass_vec_t *b) {
  return (compass_vec_t){
      .x = a->y * b->z - a->z * b->y,
      .y = a->z * b->x - a->x * b->z,
      .z = a->x * b->y - a->y * b->x,
  };
}

static float norm(const compass_vec_t *v) { return sqrtf(dot(v, v)); }

static void scale(compass_vec_t *v, float factor) {
  v->x *= factor;
  v->y *= factor;
  v->z *= factor;
}

static void blend(compass_vec_t *v, const compass_vec_t *target,
                  float weight) {
  v->x += weight * (target->x - v->x);
  v->y += weight * (target->y - v->y);
  v->z += weight * (target->z - v->z);
}

static float clamp01(float value) {
  if (value < 0.0f) {
    return 0.0f;
  }
  return (value > 1.0f) ? 1.0f : value;
}

static void rotate_with_gyro(compass_vec_t *v, const compass_vec_t *w_rad_s,
                             float dt_s) {
  float length = norm(v);
  compass_vec_t turn = cross(w_rad_s, v);
  v->x -= turn.x * dt_s;
  v->y -= turn.y * dt_s;
  v->z -= turn.z * dt_s;
  float new_length = norm(v);
  if (new_length > 0.0f) {
    scale(v, length / new_length);
  }
}

static void window_reset(attitude_still_window_t *w) {
  *w = (attitude_still_window_t){0};
}

static void track_stillness(attitude_t *att, const compass_vec_t *gyro_dps,
                            float accel_norm_g, float dt_s) {
  attitude_still_window_t *w = &att->window;
  const attitude_config_t *c = &att->config;
  if (w->samples == 0) {
    w->accel_min_g = accel_norm_g;
    w->accel_max_g = accel_norm_g;
  }
  w->samples++;
  w->elapsed_s += dt_s;
  w->gyro_sum.x += gyro_dps->x;
  w->gyro_sum.y += gyro_dps->y;
  w->gyro_sum.z += gyro_dps->z;
  w->gyro_sum_sq.x += gyro_dps->x * gyro_dps->x;
  w->gyro_sum_sq.y += gyro_dps->y * gyro_dps->y;
  w->gyro_sum_sq.z += gyro_dps->z * gyro_dps->z;
  if (accel_norm_g < w->accel_min_g) {
    w->accel_min_g = accel_norm_g;
  }
  if (accel_norm_g > w->accel_max_g) {
    w->accel_max_g = accel_norm_g;
  }
  if (w->elapsed_s < c->still_window_s) {
    return;
  }

  float n = (float)w->samples;
  compass_vec_t mean = w->gyro_sum;
  scale(&mean, 1.0f / n);
  float var_x = w->gyro_sum_sq.x / n - mean.x * mean.x;
  float var_y = w->gyro_sum_sq.y / n - mean.y * mean.y;
  float var_z = w->gyro_sum_sq.z / n - mean.z * mean.z;
  float max_var = c->still_gyro_std_dps * c->still_gyro_std_dps;
  att->still = var_x <= max_var && var_y <= max_var && var_z <= max_var &&
               (w->accel_max_g - w->accel_min_g) <= c->still_accel_range_g &&
               norm(&mean) <= c->still_max_rate_dps;
  if (att->still) {
    if (att->bias_valid) {
      blend(&att->gyro_bias_dps, &mean, c->bias_alpha);
    } else {
      att->gyro_bias_dps = mean;
      att->bias_valid = true;
    }
  }
  window_reset(w);
}

void attitude_default_config(attitude_config_t *config) {
  *config = (attitude_config_t){
      .accel_time_constant_s = 1.0f,
      .mag_time_constant_s = 1.0f,
      .accel_gate_g = 0.15f,
      .still_window_s = 1.0f,
      .still_gyro_std_dps = 0.5f,
      .still_accel_range_g = 0.03f,
      .still_max_rate_dps = 8.0f,
      .bias_alpha = 0.2f,
      .screen_axis_max_deg = 35.0f,
      .top_axis_min_deg = 55.0f,
  };
}

void attitude_init(attitude_t *att, const attitude_config_t *config) {
  *att = (attitude_t){0};
  if (config != NULL) {
    att->config = *config;
  } else {
    attitude_default_config(&att->config);
  }
  attitude_reset(att);
}

void attitude_reset(attitude_t *att) {
  att->down_valid = false;
  att->field_valid = false;
  att->still = false;
  att->axis = ATTITUDE_AXIS_SCREEN;
  window_reset(&att->window);
}

void attitude_forget_field(attitude_t *att) { att->field_valid = false; }

void attitude_update_imu(attitude_t *att, const compass_vec_t *accel_g,
                         const compass_vec_t *gyro_dps, float dt_s) {
  float accel_norm = norm(accel_g);
  track_stillness(att, gyro_dps, accel_norm, dt_s);

  compass_vec_t rate = *gyro_dps;
  if (att->bias_valid) {
    rate.x -= att->gyro_bias_dps.x;
    rate.y -= att->gyro_bias_dps.y;
    rate.z -= att->gyro_bias_dps.z;
  }
  scale(&rate, DEG_TO_RAD);
  if (att->down_valid) {
    rotate_with_gyro(&att->down, &rate, dt_s);
  }
  if (att->field_valid) {
    rotate_with_gyro(&att->field, &rate, dt_s);
  }

  if (accel_norm < MIN_ACCEL_G) {
    return;
  }
  compass_vec_t measured = *accel_g;
  scale(&measured, -1.0f / accel_norm);
  if (!att->down_valid) {
    att->down = measured;
    att->down_valid = true;
    return;
  }
  float gate = 1.0f - fabsf(accel_norm - 1.0f) / att->config.accel_gate_g;
  float weight =
      clamp01(dt_s / att->config.accel_time_constant_s) * clamp01(gate);
  blend(&att->down, &measured, weight);
  float length = norm(&att->down);
  if (length > 0.0f) {
    scale(&att->down, 1.0f / length);
  }
}

void attitude_update_mag(attitude_t *att, const compass_vec_t *field_ut,
                         float dt_s) {
  if (!att->field_valid) {
    att->field = *field_ut;
    att->field_valid = true;
    return;
  }
  blend(&att->field, field_ut,
        clamp01(dt_s / att->config.mag_time_constant_s));
}

static float elevation_deg(const compass_vec_t *axis,
                           const compass_vec_t *down) {
  float s = -dot(axis, down);
  if (s > 1.0f) {
    s = 1.0f;
  } else if (s < -1.0f) {
    s = -1.0f;
  }
  return asinf(s) * RAD_TO_DEG;
}

void attitude_output(attitude_t *att, float declination_deg,
                     float min_horizontal_ut, attitude_output_t *out) {
  *out = (attitude_output_t){.axis = att->axis};
  if (!att->down_valid) {
    return;
  }
  const compass_vec_t *down = &att->down;
  compass_vec_t up = *down;
  scale(&up, -1.0f);
  compass_tilt(&up, &out->pitch_deg, &out->roll_deg);

  static const compass_vec_t top = {1.0f, 0.0f, 0.0f};
  static const compass_vec_t screen = {0.0f, 0.0f, 1.0f};
  float screen_elevation = fabsf(elevation_deg(&screen, down));
  if (att->axis == ATTITUDE_AXIS_SCREEN &&
      screen_elevation > att->config.top_axis_min_deg) {
    att->axis = ATTITUDE_AXIS_TOP;
  } else if (att->axis == ATTITUDE_AXIS_TOP &&
             screen_elevation < att->config.screen_axis_max_deg) {
    att->axis = ATTITUDE_AXIS_SCREEN;
  }
  const compass_vec_t *axis =
      (att->axis == ATTITUDE_AXIS_SCREEN) ? &screen : &top;
  out->axis = att->axis;
  out->axis_elevation_deg = elevation_deg(axis, down);

  if (!att->field_valid) {
    return;
  }
  out->field_valid = true;
  out->field_ut = norm(&att->field);
  if (out->field_ut > 0.0f) {
    float s = dot(&att->field, down) / out->field_ut;
    out->inclination_deg = asinf(s > 1.0f ? 1.0f : (s < -1.0f ? -1.0f : s)) *
                           RAD_TO_DEG;
  }

  compass_vec_t east = cross(down, &att->field);
  float horizontal = norm(&east);
  if (horizontal < min_horizontal_ut) {
    return;
  }
  scale(&east, 1.0f / horizontal);
  compass_vec_t north = cross(&east, down);
  float magnetic =
      atan2f(dot(axis, &east), dot(axis, &north)) * RAD_TO_DEG;
  out->heading_valid = true;
  out->heading_magnetic_deg = compass_wrap_deg(magnetic);
  out->heading_true_deg = compass_wrap_deg(magnetic + declination_deg);
}
