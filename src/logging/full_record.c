#include "full_record.h"
#include "csv.h"

// #F columns (tools/tclog.py writes the same names as the CSV header):
//   utc,time_ok,t_ms,
//   temp_c,rh_pct,press_hpa,
//   gps,fixq,lat,lon,alt_m,spd_kmh,crs_deg,sats,hdop,sats_view,cn0,
//   baro_alt_m,alt_ref,asc_m,desc_m,
//   hdg_true,hdg_mag,hdg_ref,pitch_deg,roll_deg,field_ut,incl_deg,
//   mag_x_ut,mag_y_ut,mag_z_ut,
//   acc_x_g,acc_y_g,acc_z_g,gyr_x_dps,gyr_y_dps,gyr_z_dps,imu_frames,imu_ovf,
//   imu_temp_c,
//   steps,cadence_spm,walking,track_m,trip,trip_steps,trip_m,
//   pos_src,east_m,north_m,sigma_m,pos_lat,pos_lon,dr_east_m,dr_north_m

static void append_text(csv_t *csv, bool valid, const char *text) {
  csv_append(csv, ",%s", valid ? text : "");
}

static void append_gps(csv_t *csv, const full_record_t *r) {
  bool fix = (r->gps == SAMPLE_GPS_FIX);
  bool receiving = fix || (r->gps == SAMPLE_GPS_NO_FIX);
  csv_append(csv, ",%s", sample_gps_text(r->gps));
  if (receiving) {
    csv_append(csv, ",%u", r->fix_quality);
  } else {
    csv_append(csv, ",");
  }
  csv_degrees(csv, fix, r->latitude_e7);
  csv_degrees(csv, fix, r->longitude_e7);
  if (fix && r->altitude_valid) {
    csv_scaled(csv, r->altitude_dm, 1);
  } else {
    csv_append(csv, ",");
  }
  if (fix && r->speed_valid) {
    csv_scaled(csv, (int32_t)r->speed_kmh_x10, 1);
  } else {
    csv_append(csv, ",");
  }
  if (fix && r->course_valid) {
    csv_scaled(csv, r->course_x100, 2);
  } else {
    csv_append(csv, ",");
  }
  if (receiving) {
    csv_append(csv, ",%u", r->satellites);
  } else {
    csv_append(csv, ",");
  }
  if (fix && r->hdop_valid) {
    csv_scaled(csv, r->hdop_x100, 2);
  } else {
    csv_append(csv, ",");
  }
  if (receiving && r->sky_valid) {
    csv_append(csv, ",%u,%u", r->satellites_in_view, r->cno_top4_avg);
  } else {
    csv_append(csv, ",,");
  }
}

static void append_attitude(csv_t *csv, const full_record_t *r) {
  csv_float(csv, r->heading_valid, r->heading_true_deg, 1);
  csv_float(csv, r->heading_valid, r->heading_magnetic_deg, 1);
  static const char *const refs[] = {
      [FULL_HDG_REF_NONE] = "",
      [FULL_HDG_REF_BACK] = "B",
      [FULL_HDG_REF_TOP] = "T",
  };
  append_text(csv, r->heading_valid, refs[r->heading_ref]);
  csv_float(csv, r->tilt_valid, r->pitch_deg, 1);
  csv_float(csv, r->tilt_valid, r->roll_deg, 1);
  csv_float(csv, r->field_valid, r->field_ut, 1);
  csv_float(csv, r->field_valid, r->inclination_deg, 1);
  for (uint8_t i = 0; i < 3; i++) {
    csv_float(csv, r->mag_valid, r->mag_ut[i], 2);
  }
}

static void append_imu(csv_t *csv, const full_record_t *r) {
  bool frames = r->imu_frames > 0;
  for (uint8_t i = 0; i < 3; i++) {
    csv_float(csv, frames, r->accel_g[i], 4);
  }
  for (uint8_t i = 0; i < 3; i++) {
    csv_float(csv, frames, r->gyro_dps[i], 3);
  }
  csv_append(csv, ",%lu,%lu", (unsigned long)r->imu_frames,
             (unsigned long)r->imu_overflows);
  csv_float(csv, r->imu_temp_valid, r->imu_temp_c, 1);
}

static void append_motion(csv_t *csv, const full_record_t *r) {
  static const char trips[] = {
      [FULL_TRIP_STOPPED] = 'S',
      [FULL_TRIP_RUNNING] = 'R',
      [FULL_TRIP_PAUSED] = 'P',
  };
  csv_append(csv, ",%lu", (unsigned long)r->steps);
  csv_float(csv, true, r->cadence_spm, 1);
  csv_append(csv, ",%u", r->walking ? 1u : 0u);
  csv_float(csv, true, r->track_m, 1);
  csv_append(csv, ",%c,%lu", trips[r->trip], (unsigned long)r->trip_steps);
  csv_float(csv, true, r->trip_m, 1);
}

static void append_position(csv_t *csv, const full_record_t *r) {
  static const char *const sources[] = {
      [FULL_POS_NONE] = "none",
      [FULL_POS_PDR] = "pdr",
      [FULL_POS_GPS_PDR] = "gps",
  };
  bool pos = (r->pos_source != FULL_POS_NONE);
  csv_append(csv, ",%s", sources[r->pos_source]);
  csv_float(csv, pos, r->east_m, 1);
  csv_float(csv, pos, r->north_m, 1);
  csv_float(csv, pos, r->sigma_m, 1);
  csv_degrees(csv, r->pos_latlon_valid, r->pos_lat_e7);
  csv_degrees(csv, r->pos_latlon_valid, r->pos_lon_e7);
  csv_float(csv, true, r->dr_east_m, 1);
  csv_float(csv, true, r->dr_north_m, 1);
}

size_t full_record_to_csv(const full_record_t *r, char *out, size_t size) {
  csv_t csv;
  csv_init(&csv, out, size);
  csv_append(&csv, "#F,");
  sample_append_utc(&csv, &r->utc, r->time_valid);
  csv_append(&csv, ",%lu", (unsigned long)r->t_ms);

  csv_float(&csv, r->climate_valid, r->temperature_c, 2);
  csv_float(&csv, r->climate_valid, r->humidity_pct, 2);
  csv_float(&csv, r->pressure_valid, r->pressure_pa / 100.0f, 2);

  append_gps(&csv, r);

  csv_float(&csv, r->baro_valid, r->baro_altitude_m, 1);
  append_text(&csv, r->baro_valid, r->baro_gps_ref ? "G" : "S");
  csv_float(&csv, r->baro_valid, r->ascent_m, 1);
  csv_float(&csv, r->baro_valid, r->descent_m, 1);

  append_attitude(&csv, r);
  append_imu(&csv, r);
  append_motion(&csv, r);
  append_position(&csv, r);
  csv_append(&csv, "\n");
  return csv_length(&csv);
}

size_t full_imu_to_csv(uint32_t t_ms, const int16_t raw[6], char *out,
                       size_t size) {
  csv_t csv;
  csv_init(&csv, out, size);
  csv_append(&csv, "#I,%lu,%d,%d,%d,%d,%d,%d\n", (unsigned long)t_ms, raw[0],
             raw[1], raw[2], raw[3], raw[4], raw[5]);
  return csv_length(&csv);
}

size_t full_meta_to_csv(const full_meta_t *m, char *out, size_t size) {
  csv_t csv;
  csv_init(&csv, out, size);
  csv_append(&csv, "#H,");
  sample_append_utc(&csv, &m->utc, m->time_valid);
  csv_append(&csv, ",%lu,%08lx,%lu,%lu", (unsigned long)m->t_ms,
             (unsigned long)m->session, (unsigned long)m->period_ms,
             (unsigned long)m->imu_period_us);
  csv_float(&csv, true, m->accel_lsb_per_g, 1);
  csv_float(&csv, true, m->gyro_lsb_per_dps, 2);
  csv_append(&csv, ",%d,%d,%d,%d,%d,%d", m->imu_axes[0], m->imu_axes[1],
             m->imu_axes[2], m->mag_axes[0], m->mag_axes[1], m->mag_axes[2]);
  csv_float(&csv, true, m->declination_deg, 1);
  csv_append(&csv, "\n");
  return csv_length(&csv);
}
