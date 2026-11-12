#include "pdr.h"
#include <math.h>

#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f

void pdr_reset(pdr_t *pdr) { *pdr = (pdr_t){0}; }

void pdr_add(pdr_t *pdr, uint32_t steps, float distance_m, bool heading_valid,
             float heading_deg) {
  if (steps == 0) {
    return;
  }
  if (!heading_valid) {
    pdr->steps_no_heading += steps;
    return;
  }
  float heading_rad = heading_deg * DEG_TO_RAD;
  pdr->east_m += distance_m * sinf(heading_rad);
  pdr->north_m += distance_m * cosf(heading_rad);
  pdr->distance_m += distance_m;
  pdr->steps += steps;
}

float pdr_range_m(const pdr_t *pdr) {
  return sqrtf(pdr->east_m * pdr->east_m + pdr->north_m * pdr->north_m);
}

bool pdr_bearing_deg(const pdr_t *pdr, float min_range_m, float *bearing_deg) {
  if (pdr_range_m(pdr) < min_range_m) {
    return false;
  }
  float bearing = atan2f(pdr->east_m, pdr->north_m) * RAD_TO_DEG;
  if (bearing < 0.0f) {
    bearing += 360.0f;
  }
  *bearing_deg = bearing;
  return true;
}
