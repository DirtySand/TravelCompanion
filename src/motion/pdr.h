#ifndef PDR_H
#define PDR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float east_m;
  float north_m;
  float distance_m;
  uint32_t steps;
  uint32_t steps_no_heading;
} pdr_t;

void pdr_reset(pdr_t *pdr);

void pdr_add(pdr_t *pdr, uint32_t steps, float distance_m, bool heading_valid,
             float heading_deg);

float pdr_range_m(const pdr_t *pdr);

bool pdr_bearing_deg(const pdr_t *pdr, float min_range_m, float *bearing_deg);

#endif // PDR_H
