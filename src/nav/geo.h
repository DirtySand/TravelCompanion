#ifndef GEO_H
#define GEO_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool valid;
  int32_t lat_e7;
  int32_t lon_e7;
  float m_per_deg_lat;
  float m_per_deg_lon;
} geo_origin_t;

void geo_origin_set(geo_origin_t *origin, int32_t lat_e7, int32_t lon_e7);

void geo_to_local(const geo_origin_t *origin, int32_t lat_e7, int32_t lon_e7,
                  float *east_m, float *north_m);
void geo_from_local(const geo_origin_t *origin, float east_m, float north_m,
                    int32_t *lat_e7, int32_t *lon_e7);

float geo_bearing_deg(float east_m, float north_m);

float geo_angle_delta(float target_deg, float from_deg);

#endif // GEO_H
