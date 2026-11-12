#ifndef SUN_H
#define SUN_H

#include <stdbool.h>
#include <stdint.h>

#define SUN_ZENITH_SUNRISE_DEG 90.833
#define SUN_ZENITH_CIVIL_DEG 96.0

typedef enum {
  SUN_EVENT_OK,
  SUN_EVENT_ALWAYS_UP,
  SUN_EVENT_ALWAYS_DOWN,
} sun_event_status_t;

typedef struct {
  int64_t noon_s;
  sun_event_status_t status;
  int64_t rise_s;
  int64_t set_s;
  sun_event_status_t civil_status;
  int64_t dawn_s;
  int64_t dusk_s;
} sun_day_t;

void sun_day(int64_t ref_s, double lat_deg, double lon_deg, sun_day_t *day);

void sun_position(int64_t t_s, double lat_deg, double lon_deg,
                  double *elevation_deg, double *azimuth_deg);

#endif // SUN_H
