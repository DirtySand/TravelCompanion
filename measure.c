#include "measure.h"
#include "geo.h"
#include <math.h>

float measure_polygon_area_m2(const measure_point_t *points, uint16_t count) {
  if (count < 3) {
    return 0.0f;
  }
  double sum = 0.0;
  for (uint16_t i = 1; i + 1 < count; i++) {
    double x1 = points[i].east_m - points[0].east_m;
    double y1 = points[i].north_m - points[0].north_m;
    double x2 = points[i + 1].east_m - points[0].east_m;
    double y2 = points[i + 1].north_m - points[0].north_m;
    sum += x1 * y2 - x2 * y1;
  }
  return (float)fabs(sum / 2.0);
}

float measure_distance_m(measure_point_t a, measure_point_t b) {
  float de = b.east_m - a.east_m;
  float dn = b.north_m - a.north_m;
  return sqrtf(de * de + dn * dn);
}

float measure_polygon_perimeter_m(const measure_point_t *points,
                                  uint16_t count) {
  if (count < 2) {
    return 0.0f;
  }
  float sum = 0.0f;
  for (uint16_t i = 0; i < count; i++) {
    sum += measure_distance_m(points[i], points[(i + 1) % count]);
  }
  return sum;
}

static float cross(measure_point_t o, measure_point_t a, measure_point_t b) {
  return (a.east_m - o.east_m) * (b.north_m - o.north_m) -
         (a.north_m - o.north_m) * (b.east_m - o.east_m);
}

static bool segments_cross(measure_point_t p1, measure_point_t p2,
                           measure_point_t q1, measure_point_t q2) {
  float d1 = cross(q1, q2, p1);
  float d2 = cross(q1, q2, p2);
  float d3 = cross(p1, p2, q1);
  float d4 = cross(p1, p2, q2);
  return ((d1 > 0.0f) != (d2 > 0.0f)) && ((d3 > 0.0f) != (d4 > 0.0f)) &&
         d1 != 0.0f && d2 != 0.0f && d3 != 0.0f && d4 != 0.0f;
}

bool measure_polygon_self_intersects(const measure_point_t *points,
                                     uint16_t count) {
  if (count < 4) {
    return false;
  }
  for (uint16_t i = 0; i < count; i++) {
    measure_point_t a1 = points[i];
    measure_point_t a2 = points[(i + 1) % count];
    for (uint16_t j = (uint16_t)(i + 2); j < count; j++) {
      if (i == 0 && j == count - 1) {
        continue;
      }
      if (segments_cross(a1, a2, points[j], points[(j + 1) % count])) {
        return true;
      }
    }
  }
  return false;
}

bool measure_line_from(measure_point_t a, measure_point_t b,
                       float min_length_m, measure_line_t *line) {
  float length = measure_distance_m(a, b);
  if (length < min_length_m || length <= 0.0f) {
    return false;
  }
  line->origin = a;
  line->dir_east = (b.east_m - a.east_m) / length;
  line->dir_north = (b.north_m - a.north_m) / length;
  line->bearing_deg = geo_bearing_deg(line->dir_east, line->dir_north);
  return true;
}

void measure_line_project(const measure_line_t *line, measure_point_t p,
                          float *along_m, float *offset_m) {
  float de = p.east_m - line->origin.east_m;
  float dn = p.north_m - line->origin.north_m;
  *along_m = de * line->dir_east + dn * line->dir_north;
  *offset_m = de * line->dir_north - dn * line->dir_east;
}
