#include "fusion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define LAT_E7 502600000
#define LON_E7 190200000
#define FAR_LAT_E7 502650000

int main(void) {
  fusion_t f;
  fusion_init(&f, NULL);
  assert(fusion_update_gps(&f, LAT_E7, LON_E7, 1.0f, 0.0f));
  assert(fabsf(f.east_m) < 0.01f && fabsf(f.north_m) < 0.01f);

  assert(!fusion_update_gps(&f, FAR_LAT_E7, LON_E7, 1.0f, 0.0f));
  assert(fabsf(f.north_m) < 0.01f && f.rejected == 1);

  fusion_invalidate_position(&f);
  assert(fusion_update_gps(&f, FAR_LAT_E7, LON_E7, 1.0f, 0.0f));
  assert(f.north_m > 500.0f && f.north_m < 600.0f);
  assert(!f.position_lost && f.resets == 1);

  int32_t lat, lon;
  assert(fusion_lat_lon(&f, &lat, &lon));
  assert(labs((long)lat - FAR_LAT_E7) < 5 && labs((long)lon - LON_E7) < 5);

  puts("fusion ok");
  return 0;
}
