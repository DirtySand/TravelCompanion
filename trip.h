#ifndef TRIP_H
#define TRIP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  TRIP_STOPPED,
  TRIP_RUNNING,
  TRIP_PAUSED,
} trip_state_t;

typedef struct {
  trip_state_t state;
  uint64_t elapsed_us;
  uint64_t started_us;
  uint32_t steps;
  float distance_m;
} trip_t;

void trip_reset(trip_t *trip);
trip_state_t trip_toggle(trip_t *trip, uint64_t now_us);
bool trip_running(const trip_t *trip);
void trip_add(trip_t *trip, uint32_t steps, float distance_m);
uint64_t trip_elapsed_us(const trip_t *trip, uint64_t now_us);
float trip_average_speed(const trip_t *trip, uint64_t now_us);

#endif // TRIP_H
