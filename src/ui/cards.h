#ifndef CARDS_H
#define CARDS_H

#include "ui.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  CARDS_SOURCE_NONE,
  CARDS_SOURCE_PDR,
  CARDS_SOURCE_GPS_PDR,
  CARDS_SOURCE_GPS,
} cards_source_t;

typedef struct {
  cards_source_t source;
  bool latlon_valid;
  int32_t lat_e7;
  int32_t lon_e7;
  float sigma_m;
  bool gps_on;
  bool gps_fix;
  uint8_t satellites;
  float hdop;
  uint32_t gps_used;
  uint32_t gps_rejected;
  bool altitude_valid;
  bool altitude_gps_ref;
  float altitude_m;
  float track_m;
  bool heading_valid;
  float heading_deg;
} cards_nav_t;

typedef struct {
  ui_trip_state_t state;
  bool valid;
  float altitude_m;
  bool gps_ref;
  float pressure_hpa;
  float ascent_m;
  float descent_m;
  float horizontal_m;
  float slope_m;
  float threshold_m;
} cards_alt_t;

typedef struct {
  bool storage_ok;
  uint16_t count;
  uint16_t free_slots;
  bool selecting;
  bool clear_selected;
  bool has_item;
  uint16_t id;
  bool latlon_valid;
  int32_t lat_e7;
  int32_t lon_e7;
  bool altitude_valid;
  float altitude_m;
  bool relative_valid;
  float distance_m;
  float bearing_deg;
  bool turn_valid;
  float turn_deg;
  bool dalt_valid;
  float dalt_m;
  bool time_valid;
  uint8_t hour, minute, day, month;
} cards_wpt_t;

typedef struct {
  char a_label[8];
  char b_label[8];
  uint8_t selecting;
  bool valid;
  float horizontal_m;
  bool dalt_valid;
  float dalt_m;
  bool track_valid;
  float track_m;
  float bearing_deg;
  const char *guide;
} cards_dist_t;

typedef enum {
  CARDS_LINE_SET_A,
  CARDS_LINE_SET_B,
  CARDS_LINE_ACTIVE,
} cards_line_stage_t;

typedef struct {
  cards_line_stage_t stage;
  bool position_ok;
  float from_a_m;
  float min_base_m;
  float bearing_deg;
  float base_m;
  float along_m;
  float offset_m;
  bool heading_valid;
  float heading_deg;
} cards_line_t;

typedef struct {
  uint16_t count;
  uint16_t max;
  float area_m2;
  float perimeter_m;
  bool last_valid;
  float last_edge_m;
  bool here_valid;
  float to_first_m;
  float area_with_here_m2;
  bool crossing;
  cards_source_t source;
} cards_area_t;

typedef enum {
  CARDS_SUN_OK,
  CARDS_SUN_ALWAYS_UP,
  CARDS_SUN_ALWAYS_DOWN,
} cards_sun_status_t;

typedef struct {
  bool time_valid;
  bool position_valid;
  const char *position_source;
  cards_sun_status_t status;
  uint16_t rise_min;
  uint16_t set_min;
  uint16_t noon_min;
  uint16_t day_length_min;
  bool to_set;
  bool countdown_valid;
  uint32_t countdown_min;
  cards_sun_status_t civil_status;
  uint16_t dawn_min;
  uint16_t dusk_min;
  float elevation_deg;
  float azimuth_deg;
} cards_sun_t;

void cards_nav(const cards_nav_t *c, ui_row_t *rows);
void cards_alt(const cards_alt_t *c, ui_row_t *rows);
void cards_wpt(const cards_wpt_t *c, ui_row_t *rows);
void cards_dist(const cards_dist_t *c, ui_row_t *rows);
void cards_line(const cards_line_t *c, ui_row_t *rows);
void cards_area(const cards_area_t *c, ui_row_t *rows);
void cards_sun(const cards_sun_t *c, ui_row_t *rows);

#endif // CARDS_H
