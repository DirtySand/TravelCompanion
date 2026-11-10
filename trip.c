#include "trip.h"

#define US_PER_S 1000000.0f

void trip_reset(trip_t *trip) { *trip = (trip_t){.state = TRIP_STOPPED}; }

trip_state_t trip_toggle(trip_t *trip, uint64_t now_us) {
  if (trip->state == TRIP_RUNNING) {
    trip->elapsed_us += now_us - trip->started_us;
    trip->state = TRIP_PAUSED;
  } else {
    trip->started_us = now_us;
    trip->state = TRIP_RUNNING;
  }
  return trip->state;
}

bool trip_running(const trip_t *trip) { return trip->state == TRIP_RUNNING; }

void trip_add(trip_t *trip, uint32_t steps, float distance_m) {
  if (!trip_running(trip)) {
    return;
  }
  trip->steps += steps;
  trip->distance_m += distance_m;
}

uint64_t trip_elapsed_us(const trip_t *trip, uint64_t now_us) {
  if (trip->state == TRIP_RUNNING) {
    return trip->elapsed_us + (now_us - trip->started_us);
  }
  return trip->elapsed_us;
}

float trip_average_speed(const trip_t *trip, uint64_t now_us) {
  float seconds = (float)trip_elapsed_us(trip, now_us) / US_PER_S;
  return (seconds > 0.0f) ? trip->distance_m / seconds : 0.0f;
}
