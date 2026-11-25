#include "full_record.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static unsigned columns(const char *line) {
  unsigned count = 1;
  for (const char *p = line; *p != '\0'; p++) {
    if (*p == ',') {
      count++;
    }
  }
  return count;
}

static full_record_t worst_case(void) {
  full_record_t r = {
      .utc = {.seconds = 59,
              .minutes = 59,
              .hours = 23,
              .day = 31,
              .month = 12,
              .year = 99},
      .time_valid = true,
      .t_ms = 4294967295u,
      .climate_valid = true,
      .temperature_c = -40.25f,
      .humidity_pct = 100.0f,
      .pressure_valid = true,
      .pressure_pa = 110000.0f,
      .gps = SAMPLE_GPS_FIX,
      .fix_quality = 2,
      .latitude_e7 = -899999999,
      .longitude_e7 = -1799999999,
      .altitude_valid = true,
      .altitude_dm = -123456,
      .speed_valid = true,
      .speed_kmh_x10 = 99999,
      .course_valid = true,
      .course_x100 = 35999,
      .satellites = 12,
      .hdop_valid = true,
      .hdop_x100 = 9999,
      .sky_valid = true,
      .satellites_in_view = 16,
      .cno_top4_avg = 45,
      .baro_valid = true,
      .baro_altitude_m = -12345.6f,
      .baro_gps_ref = true,
      .ascent_m = 99999.9f,
      .descent_m = 99999.9f,
      .tilt_valid = true,
      .pitch_deg = -89.9f,
      .roll_deg = -179.9f,
      .heading_valid = true,
      .heading_true_deg = 359.9f,
      .heading_magnetic_deg = 353.4f,
      .heading_ref = FULL_HDG_REF_BACK,
      .field_valid = true,
      .field_ut = 999.9f,
      .inclination_deg = -89.9f,
      .mag_valid = true,
      .mag_ut = {-800.0f, -800.0f, -800.0f},
      .imu_frames = 4294967295u,
      .imu_overflows = 4294967295u,
      .accel_g = {-4.0f, -4.0f, -4.0f},
      .gyro_dps = {-500.0f, -500.0f, -500.0f},
      .imu_temp_valid = true,
      .imu_temp_c = -40.0f,
      .steps = 4294967295u,
      .cadence_spm = 240.0f,
      .walking = true,
      .track_m = 9999999.9f,
      .trip = FULL_TRIP_PAUSED,
      .trip_steps = 4294967295u,
      .trip_m = 9999999.9f,
      .pos_source = FULL_POS_GPS_PDR,
      .east_m = -9999999.9f,
      .north_m = -9999999.9f,
      .sigma_m = 99999.9f,
      .pos_latlon_valid = true,
      .pos_lat_e7 = -899999999,
      .pos_lon_e7 = -1799999999,
      .dr_east_m = -9999999.9f,
      .dr_north_m = -9999999.9f,
  };
  return r;
}

int main(void) {
  char line[FULL_RECORD_CSV_MAX];

  full_record_t full = worst_case();
  size_t length = full_record_to_csv(&full, line, sizeof(line));
  assert(length > 0 && length < FULL_RECORD_CSV_MAX);
  assert(strncmp(line, "#F,2099-12-31T23:59:59Z,1,4294967295,", 37) == 0);
  assert(line[length - 1] == '\n');
  assert(columns(line + 3) == FULL_RECORD_COLUMNS);
  assert(strstr(line, ",fix,2,-89.9999999,-179.9999999,-12345.6,9999.9,"
                      "359.99,12,99.99,16,45,") != NULL);
  printf("worst case #F: %zu bytes\n", length);

  full_record_t empty = {0};
  length = full_record_to_csv(&empty, line, sizeof(line));
  assert(length > 0);
  assert(columns(line + 3) == FULL_RECORD_COLUMNS);
  assert(strstr(line, ",off,,,,,,,,,,") != NULL);
  assert(strstr(line, ",none,,,,,,0.0,0.0\n") != NULL);

  full_record_t nofix = {.gps = SAMPLE_GPS_NO_FIX, .satellites = 3};
  length = full_record_to_csv(&nofix, line, sizeof(line));
  assert(strstr(line, ",nofix,0,,,,,,3,,,,") != NULL);

  assert(full_record_to_csv(&full, line, 100) == 0);

  const int16_t raw[6] = {-32768, 32767, 0, -1, 1, 8192};
  char imu[FULL_IMU_CSV_MAX];
  length = full_imu_to_csv(4294967295u, raw, imu, sizeof(imu));
  assert(strcmp(imu, "#I,4294967295,-32768,32767,0,-1,1,8192\n") == 0);
  assert(length == strlen(imu));

  full_meta_t meta = {
      .utc = {.seconds = 1,
              .minutes = 2,
              .hours = 3,
              .day = 4,
              .month = 5,
              .year = 26},
      .time_valid = true,
      .t_ms = 1234,
      .session = 0xDEADBEEFu,
      .period_ms = 1000,
      .imu_period_us = 20000,
      .accel_lsb_per_g = 8192.0f,
      .gyro_lsb_per_dps = 65.5f,
      .imu_axes = {-1, -2, 3},
      .mag_axes = {-1, -2, 3},
      .declination_deg = 6.5f,
  };
  char meta_line[FULL_META_CSV_MAX];
  full_meta_to_csv(&meta, meta_line, sizeof(meta_line));
  assert(strcmp(meta_line, "#H,2026-05-04T03:02:01Z,1,1234,deadbeef,1000,"
                           "20000,8192.0,65.50,-1,-2,3,-1,-2,3,6.5\n") == 0);

  printf("full record ok\n");
  return 0;
}
