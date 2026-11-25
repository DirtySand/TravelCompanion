#ifndef FULL_RECORD_H
#define FULL_RECORD_H

#include "ds3231.h"
#include "sample.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FULL_RECORD_CSV_MAX 512u
#define FULL_RECORD_COLUMNS 55u
#define FULL_IMU_CSV_MAX 64u
#define FULL_META_CSV_MAX 160u

typedef enum {
  FULL_HDG_REF_NONE,
  FULL_HDG_REF_BACK,
  FULL_HDG_REF_TOP,
} full_hdg_ref_t;

typedef enum {
  FULL_POS_NONE,
  FULL_POS_PDR,
  FULL_POS_GPS_PDR,
} full_pos_src_t;

typedef enum {
  FULL_TRIP_STOPPED,
  FULL_TRIP_RUNNING,
  FULL_TRIP_PAUSED,
} full_trip_t;

typedef struct {
  ds3231_datetime_t utc;
  bool time_valid;
  uint32_t t_ms;

  bool climate_valid;
  float temperature_c;
  float humidity_pct;
  bool pressure_valid;
  float pressure_pa;

  sample_gps_t gps;
  uint8_t fix_quality;
  int32_t latitude_e7;
  int32_t longitude_e7;
  bool altitude_valid;
  int32_t altitude_dm;
  bool speed_valid;
  uint32_t speed_kmh_x10;
  bool course_valid;
  uint16_t course_x100;
  uint8_t satellites;
  bool hdop_valid;
  uint16_t hdop_x100;
  bool sky_valid;
  uint8_t satellites_in_view;
  uint8_t cno_top4_avg;

  bool baro_valid;
  float baro_altitude_m;
  bool baro_gps_ref;
  float ascent_m;
  float descent_m;

  bool tilt_valid;
  float pitch_deg;
  float roll_deg;
  bool heading_valid;
  float heading_true_deg;
  float heading_magnetic_deg;
  full_hdg_ref_t heading_ref;
  bool field_valid;
  float field_ut;
  float inclination_deg;

  bool mag_valid;
  float mag_ut[3];

  uint32_t imu_frames;
  uint32_t imu_overflows;
  float accel_g[3];
  float gyro_dps[3];
  bool imu_temp_valid;
  float imu_temp_c;

  uint32_t steps;
  float cadence_spm;
  bool walking;
  float track_m;
  full_trip_t trip;
  uint32_t trip_steps;
  float trip_m;

  full_pos_src_t pos_source;
  float east_m;
  float north_m;
  float sigma_m;
  bool pos_latlon_valid;
  int32_t pos_lat_e7;
  int32_t pos_lon_e7;
  float dr_east_m;
  float dr_north_m;
} full_record_t;

typedef struct {
  ds3231_datetime_t utc;
  bool time_valid;
  uint32_t t_ms;
  uint32_t session;
  uint32_t period_ms;
  uint32_t imu_period_us;
  float accel_lsb_per_g;
  float gyro_lsb_per_dps;
  int8_t imu_axes[3];
  int8_t mag_axes[3];
  float declination_deg;
} full_meta_t;

size_t full_record_to_csv(const full_record_t *r, char *out, size_t size);
size_t full_imu_to_csv(uint32_t t_ms, const int16_t raw[6], char *out,
                       size_t size);
size_t full_meta_to_csv(const full_meta_t *m, char *out, size_t size);

#endif // FULL_RECORD_H
