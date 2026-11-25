#include "sample.h"
#include "board_config.h"
#include "csv.h"
#include <stdio.h>

static const char csv_header[] =
    "utc,time_ok,mode,temp_c,rh_pct,press_hpa,gps,lat,lon,alt_m,spd_kmh,"
    "crs_deg,sats,hdop\n";

const char *sample_gps_text(sample_gps_t gps) {
  switch (gps) {
  case SAMPLE_GPS_OFF:
    return "off";
  case SAMPLE_GPS_NO_DATA:
    return "nodata";
  case SAMPLE_GPS_NO_FIX:
    return "nofix";
  case SAMPLE_GPS_FIX:
  default:
    return "fix";
  }
}

void sample_append_utc(csv_t *csv, const ds3231_datetime_t *t,
                       bool time_valid) {
  csv_append(csv, "%04u-%02u-%02uT%02u:%02u:%02uZ,%u",
             (unsigned)(CENTURY_BASE + t->year), t->month, t->day, t->hours,
             t->minutes, t->seconds, time_valid ? 1u : 0u);
}

size_t sample_csv_header(char *out, size_t size) {
  int written = snprintf(out, size, "%s", csv_header);
  return (written < 0 || (size_t)written >= size) ? 0 : (size_t)written;
}

size_t sample_to_csv(const sample_t *s, char *out, size_t size) {
  csv_t csv;
  csv_init(&csv, out, size);
  sample_append_utc(&csv, &s->utc, s->time_valid);
  csv_append(&csv, ",%c", (s->mode == SAMPLE_MODE_BACKPACK) ? 'B' : 'M');

  csv_float(&csv, s->climate_valid, s->temperature_c, 2);
  csv_float(&csv, s->climate_valid, s->humidity_pct, 2);
  csv_float(&csv, s->pressure_valid, s->pressure_pa / 100.0f, 2);

  bool fix = (s->gps == SAMPLE_GPS_FIX);
  csv_append(&csv, ",%s", sample_gps_text(s->gps));
  csv_degrees(&csv, fix, s->latitude_e7);
  csv_degrees(&csv, fix, s->longitude_e7);
  if (fix && s->altitude_valid) {
    csv_scaled(&csv, s->altitude_dm, 1);
  } else {
    csv_append(&csv, ",");
  }
  if (fix && s->speed_valid) {
    csv_scaled(&csv, (int32_t)s->speed_kmh_x10, 1);
  } else {
    csv_append(&csv, ",");
  }
  if (fix && s->course_valid) {
    csv_scaled(&csv, s->course_x100, 2);
  } else {
    csv_append(&csv, ",");
  }
  if (s->gps == SAMPLE_GPS_NO_FIX || fix) {
    csv_append(&csv, ",%u", s->satellites);
  } else {
    csv_append(&csv, ",");
  }
  if (fix && s->hdop_valid) {
    csv_scaled(&csv, s->hdop_x100, 2);
  } else {
    csv_append(&csv, ",");
  }
  csv_append(&csv, "\n");
  return csv_length(&csv);
}
