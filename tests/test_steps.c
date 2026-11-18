#include "steps.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define SAMPLE_PERIOD_S (22.0f / 1100.0f)
#define WALK_SECONDS 120.0f
#define STEP_FREQUENCY_HZ 1.8f

static unsigned walk(steps_t *s, float *distance_m) {
  unsigned steps = 0;
  int samples = (int)(WALK_SECONDS / SAMPLE_PERIOD_S);
  for (int i = 0; i < samples; i++) {
    float t = (float)i * SAMPLE_PERIOD_S;
    float accel = 2.5f * sinf(2.0f * 3.14159265f * STEP_FREQUENCY_HZ * t);
    steps += steps_update(s, accel, SAMPLE_PERIOD_S, distance_m);
  }
  return steps;
}

int main(void) {
  steps_t fresh;
  steps_init(&fresh, NULL);
  float fresh_m = 0.0f;
  unsigned fresh_steps = walk(&fresh, &fresh_m);
  assert(fresh_steps > 200 && fresh_steps < 220);

  steps_t aged;
  steps_init(&aged, NULL);
  aged.time_us = 200ull * 3600ull * 1000000ull;
  float aged_m = 0.0f;
  unsigned aged_steps = walk(&aged, &aged_m);
  assert(aged_steps == fresh_steps);
  assert(fabsf(aged_m - fresh_m) < 0.01f);

  steps_t still;
  steps_init(&still, NULL);
  float still_m = 0.0f;
  unsigned still_steps = 0;
  for (int i = 0; i < 5000; i++) {
    still_steps += steps_update(&still, 0.05f, SAMPLE_PERIOD_S, &still_m);
  }
  assert(still_steps == 0 && !steps_walking(&still));

  printf("steps ok (%u steps, %.1f m)\n", fresh_steps, (double)fresh_m);
  return 0;
}
