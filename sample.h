#ifndef SAMPLE_H
#define SAMPLE_H

#include "drivers/ds3231/ds3231.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SAMPLE_CSV_MAX 192u

typedef enum {
  SAMPLE_GPS_OFF,
  SAMPLE_GPS_NO_DATA,
  SAMPLE_GPS_NO_FIX,
  SAMPLE_GPS_FIX,
} sample_gps_t;

typedef enum {
  SAMPLE_MODE_MANUAL,
  SAMPLE_MODE_BACKPACK,
} sample_mode_t;

typedef struct {
  ds3231_datetime_t utc; // scheduled sample time (RTC alarm)
  bool time_valid;
  sample_mode_t mode;
  bool climate_valid; // SHT40
  float temperature_c;
  float humidity_pct;
  bool pressure_valid; // BME680
  float pressure_pa;
  sample_gps_t gps;
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
} sample_t;

// Both return the line length (including '\n'), 0 if it did not fit.
size_t sample_csv_header(char *out, size_t size);
size_t sample_to_csv(const sample_t *sample, char *out, size_t size);

#endif // SAMPLE_H
