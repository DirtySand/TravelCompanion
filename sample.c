#include "sample.h"
#include "board_config.h"
#include <stdarg.h>
#include <stdio.h>

#define DEGREES_SCALE 10000000u

static const char csv_header[] =
    "utc,time_ok,mode,temp_c,rh_pct,press_hpa,gps,lat,lon,alt_m,spd_kmh,"
    "crs_deg,sats,hdop\n";

typedef struct {
  char *out;
  size_t size;
  size_t length;
  bool overflow;
} csv_t;

static void append(csv_t *csv, const char *format, ...) {
  if (csv->overflow) {
    return;
  }
  va_list args;
  va_start(args, format);
  int written = vsnprintf(csv->out + csv->length, csv->size - csv->length,
                          format, args);
  va_end(args);
  if (written < 0 || (size_t)written >= csv->size - csv->length) {
    csv->overflow = true;
    return;
  }
  csv->length += (size_t)written;
}

static void append_degrees(csv_t *csv, int32_t value_e7) {
  uint32_t magnitude =
      (value_e7 < 0) ? (uint32_t)(-(int64_t)value_e7) : (uint32_t)value_e7;
  append(csv, ",%s%lu.%07lu", (value_e7 < 0) ? "-" : "",
         (unsigned long)(magnitude / DEGREES_SCALE),
         (unsigned long)(magnitude % DEGREES_SCALE));
}

static const char *gps_text(sample_gps_t gps) {
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

size_t sample_csv_header(char *out, size_t size) {
  int written = snprintf(out, size, "%s", csv_header);
  return (written < 0 || (size_t)written >= size) ? 0 : (size_t)written;
}

size_t sample_to_csv(const sample_t *s, char *out, size_t size) {
  csv_t csv = {.out = out, .size = size};
  const ds3231_datetime_t *t = &s->utc;
  append(&csv, "%04u-%02u-%02uT%02u:%02u:%02uZ,%u,%c",
         (unsigned)(CENTURY_BASE + t->year), t->month, t->day, t->hours,
         t->minutes, t->seconds, s->time_valid ? 1u : 0u,
         (s->mode == SAMPLE_MODE_BACKPACK) ? 'B' : 'M');

  if (s->climate_valid) {
    append(&csv, ",%.2f,%.2f", (double)s->temperature_c,
           (double)s->humidity_pct);
  } else {
    append(&csv, ",,");
  }
  if (s->pressure_valid) {
    append(&csv, ",%.2f", (double)(s->pressure_pa / 100.0f));
  } else {
    append(&csv, ",");
  }

  append(&csv, ",%s", gps_text(s->gps));
  if (s->gps == SAMPLE_GPS_FIX) {
    append_degrees(&csv, s->latitude_e7);
    append_degrees(&csv, s->longitude_e7);
  } else {
    append(&csv, ",,");
  }
  bool fix = (s->gps == SAMPLE_GPS_FIX);
  if (fix && s->altitude_valid) {
    int32_t alt = s->altitude_dm;
    uint32_t magnitude = (alt < 0) ? (uint32_t)(-(int64_t)alt) : (uint32_t)alt;
    append(&csv, ",%s%lu.%lu", (alt < 0) ? "-" : "",
           (unsigned long)(magnitude / 10u), (unsigned long)(magnitude % 10u));
  } else {
    append(&csv, ",");
  }
  if (fix && s->speed_valid) {
    append(&csv, ",%lu.%lu", (unsigned long)(s->speed_kmh_x10 / 10u),
           (unsigned long)(s->speed_kmh_x10 % 10u));
  } else {
    append(&csv, ",");
  }
  if (fix && s->course_valid) {
    append(&csv, ",%u.%02u", s->course_x100 / 100u, s->course_x100 % 100u);
  } else {
    append(&csv, ",");
  }
  if (s->gps == SAMPLE_GPS_NO_FIX || fix) {
    append(&csv, ",%u", s->satellites);
  } else {
    append(&csv, ",");
  }
  if (fix && s->hdop_valid) {
    append(&csv, ",%u.%02u\n", s->hdop_x100 / 100u, s->hdop_x100 % 100u);
  } else {
    append(&csv, ",\n");
  }
  return csv.overflow ? 0 : csv.length;
}
