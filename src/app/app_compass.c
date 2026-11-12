#include "app.h"
#include "board_config.h"
#include "hardware/i2c.h"
#include "qmc5883l_port.h"
#include <stdio.h>

#define COMPASS_FILTER_ALPHA 0.4f
#define COMPASS_FILTER_RESET_MS 1000
#define COMPASS_CAL_MIN_RANGE_UT 30.0f
#define MAG_ATTITUDE_MAX_DT_S 1.0f

static const compass_axes_t mag_axes = {
    .forward = MAG_AXIS_FORWARD,
    .right = MAG_AXIS_RIGHT,
    .down = MAG_AXIS_DOWN,
};

bool mag_init(app_t *app) {
  if (!compass_axes_valid(&mag_axes)) {
    printf("MAG_AXIS_* in board_config.h must use X, Y and Z once each\n");
    return false;
  }
  if (!compass_axes_right_handed(&mag_axes)) {
    printf("MAG_AXIS_* in board_config.h are mirrored: one sign is wrong\n");
  }
  app->mag =
      qmc5883l_create(I2C_PORT, QMC5883L_I2C_ADDRESS, qmc5883l_port_i2c_read,
                      qmc5883l_port_i2c_write, qmc5883l_port_delay_ms);
  if (app->mag == NULL) {
    printf("Failed to create QMC5883L instance\n");
    return false;
  }
  uint8_t chip_id = 0;
  if (qmc5883l_read_chip_id(app->mag, &chip_id) != QMC5883L_OK) {
    printf("QMC5883L not responding\n");
    return false;
  }
  printf("QMC5883L chip ID: 0x%02X%s\n", chip_id,
         (chip_id == QMC5883L_CHIP_ID) ? "" : " (expected 0xFF)");
  const qmc5883l_config_t config = {
      .odr = QMC5883L_ODR_50HZ,
      .range = QMC5883L_RANGE_2G,
      .osr = QMC5883L_OSR_512,
  };
  if (qmc5883l_init(app->mag, &config) != QMC5883L_OK ||
      qmc5883l_start(app->mag) != QMC5883L_OK) {
    printf("QMC5883L init failed\n");
    return false;
  }
  return true;
}

void mag_set_running(app_t *app, bool running) {
  if (!app->mag_ok) {
    return;
  }
  qmc5883l_status_t status =
      running ? qmc5883l_start(app->mag) : qmc5883l_standby(app->mag);
  if (status != QMC5883L_OK) {
    printf("QMC5883L mode change failed: %d\n", status);
    app->mag_error = true;
  }
  app->mag_filter_valid = false;
}

void mag_update(app_t *app) {
  if (!app->mag_ok) {
    return;
  }
  qmc5883l_raw_t raw;
  qmc5883l_status_t status = qmc5883l_read_raw(app->mag, &raw);
  if (status == QMC5883L_NOT_READY) {
    return;
  }
  if (status == QMC5883L_I2C_ERROR || status == QMC5883L_NULL_PTR) {
    if (!app->mag_error) {
      printf("QMC5883L read failed: %d\n", status);
    }
    app->mag_error = true;
    return;
  }
  app->mag_error = false;
  app->mag_received = true;
  app->mag_overflow = (status == QMC5883L_OVERFLOW);
  if (app->mag_overflow) {
    return;
  }

  float lsb_per_ut = qmc5883l_lsb_per_ut(app->mag);
  compass_vec_t field_raw = {
      .x = (float)raw.x / lsb_per_ut,
      .y = (float)raw.y / lsb_per_ut,
      .z = (float)raw.z / lsb_per_ut,
  };
  if (app->mag_calibrating) {
    compass_cal_add(&app->mag_session, &field_raw);
  }
  compass_vec_t field;
  compass_apply_cal(&app->mag_cal, &field_raw, &field);

  compass_vec_t field_device;
  compass_to_device(&mag_axes, &field, &field_device);
  float dt_s = MAG_ATTITUDE_MAX_DT_S;
  if (app->mag_fed) {
    dt_s = (float)ms_since(app->mag_fed_at) / 1000.0f;
    if (dt_s > MAG_ATTITUDE_MAX_DT_S) {
      dt_s = MAG_ATTITUDE_MAX_DT_S;
    }
  }
  attitude_update_mag(&app->attitude, &field_device, dt_s);
  app->mag_fed = true;
  app->mag_fed_at = get_absolute_time();

  if (!app->mag_filter_valid ||
      ms_since(app->mag_sampled_at) > COMPASS_FILTER_RESET_MS) {
    app->mag_field = field;
    app->mag_filter_valid = true;
  } else {
    compass_filter(&app->mag_field, &field, COMPASS_FILTER_ALPHA);
  }
  app->mag_sampled_at = get_absolute_time();
}

void start_mag_calibration(app_t *app) {
  if (!app->mag_ok) {
    show_error(app, "No magnetometer");
    return;
  }
  compass_cal_begin(&app->mag_session);
  app->mag_calibrating = true;
  printf("Magnetometer calibration started\n");
  app->refresh_now = true;
}

void cancel_mag_calibration(app_t *app) {
  if (!app->mag_calibrating) {
    return;
  }
  app->mag_calibrating = false;
  set_status(app, "Cal cancelled", UI_COLOR_GRAY);
}

static void print_mag_calibration(const compass_cal_t *cal) {
  printf("Magnetometer calibration (RAM only). To keep it, put this in "
         "board_config.h:\n");
  printf("#define MAG_CAL_VALID 1\n");
  printf("#define MAG_CAL_AXIS_MASK 0x%X\n", cal->axis_mask);
  printf("#define MAG_CAL_OFFSET_X %.3ff\n", (double)cal->offset.x);
  printf("#define MAG_CAL_OFFSET_Y %.3ff\n", (double)cal->offset.y);
  printf("#define MAG_CAL_OFFSET_Z %.3ff\n", (double)cal->offset.z);
  printf("#define MAG_CAL_SCALE_X %.4ff\n", (double)cal->scale.x);
  printf("#define MAG_CAL_SCALE_Y %.4ff\n", (double)cal->scale.y);
  printf("#define MAG_CAL_SCALE_Z %.4ff\n", (double)cal->scale.z);
}

void finish_mag_calibration(app_t *app) {
  compass_cal_t cal;
  if (!compass_cal_finish(&app->mag_session, &mag_axes,
                          COMPASS_CAL_MIN_RANGE_UT, &cal)) {
    set_status(app, "Turn more", UI_COLOR_RED);
    return;
  }
  app->mag_cal = cal;
  app->mag_calibrating = false;
  app->mag_filter_valid = false;
  attitude_forget_field(&app->attitude);
  print_mag_calibration(&cal);
  uint8_t down = (uint8_t)COMPASS_AXIS_BIT(compass_axis_index(mag_axes.down));
  set_status(app, (cal.axis_mask & down) ? "Cal saved (3D)" : "Cal saved (2D)",
             UI_COLOR_GREEN);
}

// ---------------------------------------------------------------------------
// Compass card
// ---------------------------------------------------------------------------

static void collect_compass(app_t *app, ui_compass_t *c) {
  c->sensor_ok = app->mag_ok && !app->mag_error;
  c->data_valid = app->mag_received && app->mag_filter_valid;
  c->overflow = app->mag_overflow;
  c->calibrated = app->mag_cal.valid;
  c->declination_deg = MAG_DECLINATION_DEG;
  c->calibrating = app->mag_calibrating;

  attitude_output_t att;
  attitude_output(&app->attitude, MAG_DECLINATION_DEG,
                  COMPASS_MIN_HORIZONTAL_UT, &att);
  if (att.field_valid && app->imu_ok && !app->imu_error) {
    c->reference = (att.axis == ATTITUDE_AXIS_SCREEN) ? UI_COMPASS_REF_BACK
                                                      : UI_COMPASS_REF_TOP;
    c->heading_valid = c->data_valid && att.heading_valid;
    c->heading_true_deg = att.heading_true_deg;
    c->heading_magnetic_deg = att.heading_magnetic_deg;
    c->field_ut = att.field_ut;
    c->inclination_deg = att.inclination_deg;
  } else {
    compass_reading_t reading;
    compass_compute(&mag_axes, &app->mag_field, MAG_DECLINATION_DEG,
                    COMPASS_MIN_HORIZONTAL_UT, &reading);
    c->reference = UI_COMPASS_REF_LEVEL;
    c->heading_valid = c->data_valid && reading.heading_valid;
    c->heading_true_deg = reading.heading_true_deg;
    c->heading_magnetic_deg = reading.heading_magnetic_deg;
    c->field_ut = reading.field_ut;
    c->inclination_deg = reading.inclination_deg;
  }
  uint8_t down = (uint8_t)COMPASS_AXIS_BIT(compass_axis_index(mag_axes.down));
  c->vertical_calibrated =
      app->mag_cal.valid && (app->mag_cal.axis_mask & down) != 0;
  c->x_ut = app->mag_field.x;
  c->y_ut = app->mag_field.y;
  c->z_ut = app->mag_field.z;

  if (app->mag_calibrating) {
    compass_vec_t ranges;
    compass_cal_ranges(&app->mag_session, &ranges);
    c->range_x_ut = ranges.x;
    c->range_y_ut = ranges.y;
    c->range_z_ut = ranges.z;
    c->cal_samples = app->mag_session.samples;
  }
}

void draw_compass_card(app_t *app) {
  ui_compass_t compass = {0};
  collect_compass(app, &compass);
  ui_draw_compass_card(app->oled, &compass);
}
