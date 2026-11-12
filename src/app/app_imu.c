#include "app.h"
#include "board_config.h"
#include "hardware/i2c.h"
#include "icm20649_port.h"
#include <math.h>
#include <stdio.h>

#define IMU_FILTER_ALPHA 0.4f
#define IMU_FILTER_RESET_MS 1000
#define IMU_ACCEL_RATE_DIV 21
#define IMU_GYRO_RATE_DIV 21
#define IMU_DLPF_CFG 4
#define IMU_FIFO_PROBE false
#define STEP_DEBUG_LOG true
#define IMU_FIFO_PROBE_MS 1000
#define IMU_FIFO_FRAME_BYTES 12
#define IMU_FIFO_MAX_FRAMES (4096 / IMU_FIFO_FRAME_BYTES)
#define IMU_SAMPLE_PERIOD_S ((1.0f + IMU_GYRO_RATE_DIV) / 1100.0f)
#define IMU_FIFO_CHECK_FRAMES 10
#define STANDARD_GRAVITY_MS2 9.80665f

static const compass_axes_t imu_axes = {
    .forward = IMU_AXIS_FORWARD,
    .right = IMU_AXIS_RIGHT,
    .down = IMU_AXIS_DOWN,
};

static void imu_fifo_start(app_t *app) {
  icm20649_status_t status =
      icm20649_fifo_start(app->imu,
                          ICM20649_FIFO_ACCEL | ICM20649_FIFO_GYRO_X |
                              ICM20649_FIFO_GYRO_Y | ICM20649_FIFO_GYRO_Z,
                          true);
  app->imu_fifo_running = (status == ICM20649_OK);
  if (!app->imu_fifo_running) {
    printf("ICM-20649 FIFO start failed: %d\n", status);
  }
}

static void imu_probe_fifo(app_t *app) {
  uint16_t count = 0;
  bool overflow = false;
  icm20649_status_t status = icm20649_set_rate_dividers(app->imu, 0, 0);
  if (status == ICM20649_OK) {
    status = icm20649_fifo_start(app->imu, ICM20649_FIFO_ACCEL, true);
  }
  if (status == ICM20649_OK) {
    sleep_ms(IMU_FIFO_PROBE_MS);
    status = icm20649_fifo_count(app->imu, &count);
  }
  if (status == ICM20649_OK) {
    status = icm20649_fifo_overflowed(app->imu, &overflow);
  }
  icm20649_fifo_stop(app->imu);
  icm20649_set_rate_dividers(app->imu, IMU_ACCEL_RATE_DIV, IMU_GYRO_RATE_DIV);
  if (status != ICM20649_OK) {
    printf("ICM-20649 FIFO probe failed: %d\n", status);
    return;
  }
  printf("ICM-20649 FIFO probe: %u bytes after %u ms at ~1.1 kHz, "
         "overflow flag %d\n",
         (unsigned)count, (unsigned)IMU_FIFO_PROBE_MS, (int)overflow);
}

bool imu_init(app_t *app) {
  if (!compass_axes_valid(&imu_axes)) {
    printf("IMU_AXIS_* in board_config.h must use X, Y and Z once each\n");
    return false;
  }
  if (!compass_axes_right_handed(&imu_axes)) {
    printf("IMU_AXIS_* in board_config.h are mirrored: one sign is wrong\n");
  }
  app->imu =
      icm20649_create(I2C_PORT, ICM20649_I2C_ADDRESS, icm20649_port_i2c_read,
                      icm20649_port_i2c_write, icm20649_port_delay_ms);
  if (app->imu == NULL) {
    printf("Failed to create ICM-20649 instance\n");
    return false;
  }
  const icm20649_config_t config = {
      .accel_range = ICM20649_ACCEL_4G,
      .accel_rate_div = IMU_ACCEL_RATE_DIV,
      .accel_dlpf = IMU_DLPF_CFG,
      .gyro_range = ICM20649_GYRO_500DPS,
      .gyro_rate_div = IMU_GYRO_RATE_DIV,
      .gyro_dlpf = IMU_DLPF_CFG,
  };
  icm20649_status_t status = icm20649_init(app->imu, &config);
  if (status == ICM20649_WRONG_CHIP_ID) {
    uint8_t chip_id = 0;
    icm20649_read_chip_id(app->imu, &chip_id);
    printf("ICM-20649 wrong chip ID: 0x%02X (expected 0x%02X)\n", chip_id,
           ICM20649_CHIP_ID);
    return false;
  }
  if (status != ICM20649_OK) {
    printf("ICM-20649 init failed: %d\n", status);
    return false;
  }
  printf("ICM-20649 ready\n");
  if (IMU_FIFO_PROBE) {
    imu_probe_fifo(app);
  }
  imu_fifo_start(app);
  return true;
}

void imu_set_running(app_t *app, bool running) {
  if (!app->imu_ok) {
    return;
  }
  if (!running && app->imu_fifo_running) {
    icm20649_fifo_stop(app->imu);
    app->imu_fifo_running = false;
  }
  if (icm20649_set_sleep(app->imu, !running) != ICM20649_OK) {
    printf("ICM-20649 sleep change failed\n");
    app->imu_error = true;
  }
  app->imu_filter_valid = false;
  if (running) {
    attitude_reset(&app->attitude);
    imu_fifo_start(app);
  }
}

static int16_t be16(const uint8_t *p) {
  return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void imu_check_frames(app_t *app, float accel_norm_sum,
                             uint32_t frames) {
  if (app->imu_fifo_checked || frames < IMU_FIFO_CHECK_FRAMES) {
    return;
  }
  app->imu_fifo_checked = true;
  float mean = accel_norm_sum / (float)frames;
  printf("ICM-20649 FIFO: %lu frames, mean |a| %.2f g%s\n",
         (unsigned long)frames, (double)mean,
         (mean > 0.5f && mean < 1.5f) ? "" : " - unexpected frame layout!");
}

bool walking_heading(app_t *app, float *heading_deg,
                            ui_compass_ref_t *reference) {
  if (!app->imu_ok || app->imu_error || !app->mag_ok || app->mag_error ||
      !app->mag_filter_valid) {
    return false;
  }
  attitude_output_t att;
  attitude_output(&app->attitude, MAG_DECLINATION_DEG,
                  COMPASS_MIN_HORIZONTAL_UT, &att);
  if (!att.field_valid || !att.heading_valid) {
    return false;
  }
  *heading_deg = att.heading_true_deg;
  if (reference != NULL) {
    *reference = (att.axis == ATTITUDE_AXIS_SCREEN) ? UI_COMPASS_REF_BACK
                                                    : UI_COMPASS_REF_TOP;
  }
  return true;
}

static void log_step_peak(app_t *app, float dynamic_ms2, bool new_peak,
                          uint8_t counted) {
  static float raw_max = 0.0f;
  float magnitude = fabsf(dynamic_ms2);
  if (magnitude > raw_max) {
    raw_max = magnitude;
  }
  if (!new_peak) {
    return;
  }
  static const char *const results[] = {
      [STEPS_PEAK_TOO_SOON] = "too soon",
      [STEPS_PEAK_PENDING] = "pending",
      [STEPS_PEAK_CONFIRMED] = "confirmed",
      [STEPS_PEAK_WALKING] = "walking",
  };
  const steps_peak_t *p = &app->steps.last_peak;
  printf("step t=%.2f pk=%.2f vl=%.2f rise=%.2f int=%.2f raw=%.1f "
         "pend=%u +%u %s\n",
         (double)p->time_s, (double)p->peak, (double)p->valley,
         (double)p->rise_s, (double)p->interval_s, (double)raw_max,
         (unsigned)p->pending, (unsigned)counted, results[p->result]);
  raw_max = 0.0f;
}

static void imu_count_steps(app_t *app, const compass_vec_t *accel_g) {
  float up_g;
  if (app->attitude.down_valid) {
    const compass_vec_t *d = &app->attitude.down;
    up_g = -(accel_g->x * d->x + accel_g->y * d->y + accel_g->z * d->z);
  } else {
    up_g = sqrtf(accel_g->x * accel_g->x + accel_g->y * accel_g->y +
                 accel_g->z * accel_g->z);
  }
  float dynamic_ms2 = (up_g - 1.0f) * STANDARD_GRAVITY_MS2;
  float distance_m = 0.0f;
  uint32_t peaks_before = app->steps.last_peak.count;
  uint8_t steps = steps_update(&app->steps, dynamic_ms2, IMU_SAMPLE_PERIOD_S,
                               &distance_m);
  if (steps > 0) {
    float heading_deg = 0.0f;
    bool heading_valid = walking_heading(app, &heading_deg, NULL);
    if (trip_running(&app->trip)) {
      pdr_add(&app->pdr, steps, distance_m, heading_valid, heading_deg);
    }
    nav_steps(app, distance_m, heading_valid, heading_deg);
  }
  trip_add(&app->trip, steps, distance_m);
  if (STEP_DEBUG_LOG) {
    log_step_peak(app, dynamic_ms2,
                  app->steps.last_peak.count != peaks_before, steps);
  }
}

void imu_drain_fifo(app_t *app) {
  static uint8_t buffer[IMU_FIFO_MAX_FRAMES * IMU_FIFO_FRAME_BYTES];
  if (!app->imu_ok || !app->imu_fifo_running) {
    return;
  }
  uint16_t count = 0;
  bool overflow = false;
  icm20649_status_t status = icm20649_fifo_count(app->imu, &count);
  if (status == ICM20649_OK) {
    status = icm20649_fifo_overflowed(app->imu, &overflow);
  }
  uint16_t frames = count / IMU_FIFO_FRAME_BYTES;
  if (frames > IMU_FIFO_MAX_FRAMES) {
    frames = IMU_FIFO_MAX_FRAMES;
  }
  if (status == ICM20649_OK && frames > 0) {
    status = icm20649_fifo_read(app->imu, buffer,
                                (uint16_t)(frames * IMU_FIFO_FRAME_BYTES));
  }
  if (status != ICM20649_OK) {
    if (!app->imu_error) {
      printf("ICM-20649 FIFO read failed: %d\n", status);
    }
    app->imu_error = true;
    return;
  }

  float lsb_per_g = icm20649_accel_lsb_per_g(app->imu);
  float lsb_per_dps = icm20649_gyro_lsb_per_dps(app->imu);
  float accel_norm_sum = 0.0f;
  for (uint16_t i = 0; i < frames; i++) {
    const uint8_t *frame = &buffer[i * IMU_FIFO_FRAME_BYTES];
    compass_vec_t accel = {
        .x = (float)be16(&frame[0]) / lsb_per_g,
        .y = (float)be16(&frame[2]) / lsb_per_g,
        .z = (float)be16(&frame[4]) / lsb_per_g,
    };
    compass_vec_t gyro = {
        .x = (float)be16(&frame[6]) / lsb_per_dps,
        .y = (float)be16(&frame[8]) / lsb_per_dps,
        .z = (float)be16(&frame[10]) / lsb_per_dps,
    };
    accel_norm_sum +=
        sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    compass_to_device(&imu_axes, &accel, &accel);
    compass_to_device(&imu_axes, &gyro, &gyro);
    attitude_update_imu(&app->attitude, &accel, &gyro, IMU_SAMPLE_PERIOD_S);
    imu_count_steps(app, &accel);
  }
  imu_check_frames(app, accel_norm_sum, frames);

  if (overflow) {
    printf("ICM-20649 FIFO overflow - restarting it\n");
    imu_fifo_start(app);
  }
}

// ---------------------------------------------------------------------------
// IMU card
// ---------------------------------------------------------------------------

static void imu_update(app_t *app) {
  if (!app->imu_ok) {
    return;
  }
  icm20649_raw_t raw;
  icm20649_status_t status = icm20649_read_raw(app->imu, &raw);
  if (status != ICM20649_OK) {
    if (!app->imu_error) {
      printf("ICM-20649 read failed: %d\n", status);
    }
    app->imu_error = true;
    return;
  }
  app->imu_error = false;
  app->imu_received = true;

  float lsb_per_g = icm20649_accel_lsb_per_g(app->imu);
  float lsb_per_dps = icm20649_gyro_lsb_per_dps(app->imu);
  compass_vec_t accel = {
      .x = (float)raw.accel[0] / lsb_per_g,
      .y = (float)raw.accel[1] / lsb_per_g,
      .z = (float)raw.accel[2] / lsb_per_g,
  };
  app->imu_gyro_dps = (compass_vec_t){
      .x = (float)raw.gyro[0] / lsb_per_dps,
      .y = (float)raw.gyro[1] / lsb_per_dps,
      .z = (float)raw.gyro[2] / lsb_per_dps,
  };
  app->imu_temp_c = icm20649_temperature_c(raw.temperature);
  if (!app->imu_filter_valid ||
      ms_since(app->imu_sampled_at) > IMU_FILTER_RESET_MS) {
    app->imu_accel_g = accel;
    app->imu_filter_valid = true;
  } else {
    compass_filter(&app->imu_accel_g, &accel, IMU_FILTER_ALPHA);
  }
  app->imu_sampled_at = get_absolute_time();
}

static void collect_imu(const app_t *app, ui_imu_t *imu) {
  imu->sensor_ok = app->imu_ok && !app->imu_error;
  imu->data_valid = app->imu_received && app->imu_filter_valid;
  imu->axes_ok = compass_axes_right_handed(&imu_axes);
  const compass_vec_t *a = &app->imu_accel_g;
  const compass_vec_t *g = &app->imu_gyro_dps;
  imu->accel_g[0] = a->x;
  imu->accel_g[1] = a->y;
  imu->accel_g[2] = a->z;
  imu->accel_norm_g = sqrtf(a->x * a->x + a->y * a->y + a->z * a->z);
  imu->gyro_dps[0] = g->x;
  imu->gyro_dps[1] = g->y;
  imu->gyro_dps[2] = g->z;
  imu->chip_temp_c = app->imu_temp_c;
  if (imu->axes_ok) {
    compass_vec_t device;
    compass_to_device(&imu_axes, a, &device);
    compass_tilt(&device, &imu->pitch_deg, &imu->roll_deg);
  }
}

void draw_imu_card(app_t *app) {
  imu_update(app);
  ui_imu_t imu = {0};
  collect_imu(app, &imu);
  ui_draw_imu_card(app->oled, &imu);
}
