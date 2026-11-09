#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "compass.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float accel_time_constant_s;
  float mag_time_constant_s;
  float accel_gate_g;
  float still_window_s;
  float still_gyro_std_dps;
  float still_accel_range_g;
  float still_max_rate_dps;
  float bias_alpha;
  float screen_axis_max_deg;
  float top_axis_min_deg;
} attitude_config_t;

typedef enum {
  ATTITUDE_AXIS_TOP,
  ATTITUDE_AXIS_SCREEN,
} attitude_axis_t;

typedef struct {
  uint32_t samples;
  float elapsed_s;
  compass_vec_t gyro_sum;
  compass_vec_t gyro_sum_sq;
  float accel_min_g;
  float accel_max_g;
} attitude_still_window_t;

typedef struct {
  attitude_config_t config;
  bool down_valid;
  compass_vec_t down;
  bool field_valid;
  compass_vec_t field;
  bool bias_valid;
  compass_vec_t gyro_bias_dps;
  bool still;
  attitude_still_window_t window;
  attitude_axis_t axis;
} attitude_t;

typedef struct {
  bool heading_valid;
  attitude_axis_t axis;
  float heading_magnetic_deg;
  float heading_true_deg;
  float axis_elevation_deg;
  bool field_valid;
  float field_ut;
  float inclination_deg;
  float pitch_deg;
  float roll_deg;
} attitude_output_t;

void attitude_default_config(attitude_config_t *config);
void attitude_init(attitude_t *att, const attitude_config_t *config);
void attitude_reset(attitude_t *att);
void attitude_forget_field(attitude_t *att);

void attitude_update_imu(attitude_t *att, const compass_vec_t *accel_g,
                         const compass_vec_t *gyro_dps, float dt_s);

void attitude_update_mag(attitude_t *att, const compass_vec_t *field_ut,
                         float dt_s);

void attitude_output(attitude_t *att, float declination_deg,
                     float min_horizontal_ut, attitude_output_t *out);

#endif // ATTITUDE_H
