#ifndef MEASURE_H
#define MEASURE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float east_m;
  float north_m;
} measure_point_t;

float measure_polygon_area_m2(const measure_point_t *points, uint16_t count);
float measure_polygon_perimeter_m(const measure_point_t *points,
                                  uint16_t count);

bool measure_polygon_self_intersects(const measure_point_t *points,
                                     uint16_t count);

float measure_distance_m(measure_point_t a, measure_point_t b);

typedef struct {
  measure_point_t origin;
  float dir_east;
  float dir_north;
  float bearing_deg;
} measure_line_t;

bool measure_line_from(measure_point_t a, measure_point_t b,
                       float min_length_m, measure_line_t *line);
void measure_line_project(const measure_line_t *line, measure_point_t p,
                          float *along_m, float *offset_m);

#endif // MEASURE_H
