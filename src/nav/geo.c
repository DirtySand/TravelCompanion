#include "geo.h"
#include <math.h>

#define WGS84_A 6378137.0
#define WGS84_E2 6.69437999014e-3
#define DEG_TO_RAD (3.14159265358979323846 / 180.0)
#define E7 1e7

void geo_origin_set(geo_origin_t *origin, int32_t lat_e7, int32_t lon_e7) {
  double lat = (double)lat_e7 / E7 * DEG_TO_RAD;
  double s = sin(lat);
  double w = 1.0 - WGS84_E2 * s * s;
  double meridional = WGS84_A * (1.0 - WGS84_E2) / (w * sqrt(w));
  double prime_vertical = WGS84_A / sqrt(w);
  origin->valid = true;
  origin->lat_e7 = lat_e7;
  origin->lon_e7 = lon_e7;
  origin->m_per_deg_lat = (float)(meridional * DEG_TO_RAD);
  origin->m_per_deg_lon = (float)(prime_vertical * cos(lat) * DEG_TO_RAD);
}

void geo_to_local(const geo_origin_t *origin, int32_t lat_e7, int32_t lon_e7,
                  float *east_m, float *north_m) {
  int64_t dlat = (int64_t)lat_e7 - origin->lat_e7;
  int64_t dlon = (int64_t)lon_e7 - origin->lon_e7;
  if (dlon > 1800000000LL) {
    dlon -= 3600000000LL;
  } else if (dlon < -1800000000LL) {
    dlon += 3600000000LL;
  }
  *north_m = (float)((double)dlat / E7 * origin->m_per_deg_lat);
  *east_m = (float)((double)dlon / E7 * origin->m_per_deg_lon);
}

void geo_from_local(const geo_origin_t *origin, float east_m, float north_m,
                    int32_t *lat_e7, int32_t *lon_e7) {
  double lat = origin->lat_e7 + (double)north_m / origin->m_per_deg_lat * E7;
  double lon = origin->lon_e7 + (double)east_m / origin->m_per_deg_lon * E7;
  if (lon > 1800000000.0) {
    lon -= 3600000000.0;
  } else if (lon < -1800000000.0) {
    lon += 3600000000.0;
  }
  *lat_e7 = (int32_t)llround(lat);
  *lon_e7 = (int32_t)llround(lon);
}

float geo_bearing_deg(float east_m, float north_m) {
  float bearing = atan2f(east_m, north_m) * (float)(180.0 / 3.14159265358979323846);
  return (bearing < 0.0f) ? bearing + 360.0f : bearing;
}

float geo_angle_delta(float target_deg, float from_deg) {
  float delta = fmodf(target_deg - from_deg, 360.0f);
  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta < -180.0f) {
    delta += 360.0f;
  }
  return delta;
}
