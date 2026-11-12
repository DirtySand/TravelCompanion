#include "cards.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static void clear_rows(ui_row_t *rows) {
  for (uint8_t i = 0; i < UI_CONTENT_ROWS; i++) {
    rows[i].text[0] = '\0';
    rows[i].color = UI_COLOR_WHITE;
  }
}

static void set_row(ui_row_t *rows, uint8_t row, uint16_t color,
                    const char *format, ...) {
  char buffer[48];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  strncpy(rows[row].text, buffer, UI_COLUMNS);
  rows[row].text[UI_COLUMNS] = '\0';
  rows[row].color = color;
}

static void coordinate(char *buffer, size_t size, int32_t value_e7,
                       char positive, char negative) {
  uint32_t magnitude =
      (value_e7 < 0) ? (uint32_t)(-(int64_t)value_e7) : (uint32_t)value_e7;
  snprintf(buffer, size, "%c %lu.%06lu", (value_e7 < 0) ? negative : positive,
           (unsigned long)(magnitude / 10000000u),
           (unsigned long)((magnitude % 10000000u) / 10u));
}

static void distance_row(ui_row_t *rows, uint8_t row, const char *label,
                         float metres, uint16_t color) {
  char value[16];
  size_t room = UI_COLUMNS - strlen(label) - 1u;
  if (metres < 99.95f) {
    snprintf(value, sizeof(value), "%.1f m", (double)metres);
  } else if (metres < 999.5f) {
    snprintf(value, sizeof(value), "%.0f m", (double)metres);
  } else {
    for (int decimals = 3; decimals >= 0; decimals--) {
      snprintf(value, sizeof(value), "%.*f km", decimals,
               (double)(metres / 1000.0f));
      if (strlen(value) <= room) {
        break;
      }
    }
  }
  set_row(rows, row, color, "%s %s", label, value);
}

static void bearing_row(ui_row_t *rows, uint8_t row, const char *label,
                        float degrees) {
  set_row(rows, row, UI_COLOR_WHITE, "%s %u %s", label,
          ui_round_degrees(degrees), ui_cardinal(degrees));
}

static void turn_row(ui_row_t *rows, uint8_t row, float turn_deg) {
  long degrees = lroundf(fabsf(turn_deg));
  if (degrees <= 10) {
    set_row(rows, row, UI_COLOR_GREEN, "Turn: ahead");
  } else {
    set_row(rows, row, UI_COLOR_WHITE, "Turn: %c %ld",
            (turn_deg < 0.0f) ? 'L' : 'R', degrees);
  }
}

static void signed_m_row(ui_row_t *rows, uint8_t row, const char *label,
                         float metres) {
  set_row(rows, row, UI_COLOR_WHITE, "%s %+.1f m", label, (double)metres);
}

static const char *const trip_states[] = {
    [UI_TRIP_STOPPED] = "STOP",
    [UI_TRIP_RUNNING] = "RUN",
    [UI_TRIP_PAUSED] = "PAUSE",
};
static const uint16_t trip_colors[] = {
    [UI_TRIP_STOPPED] = UI_COLOR_GRAY,
    [UI_TRIP_RUNNING] = UI_COLOR_GREEN,
    [UI_TRIP_PAUSED] = UI_COLOR_YELLOW,
};

static const char *source_text(cards_source_t source) {
  switch (source) {
  case CARDS_SOURCE_GPS_PDR:
    return "GPS+steps";
  case CARDS_SOURCE_PDR:
    return "steps only";
  case CARDS_SOURCE_GPS:
    return "GPS only";
  default:
    return "none";
  }
}

void cards_nav(const cards_nav_t *c, ui_row_t *rows) {
  clear_rows(rows);
  char buffer[24];
  static const uint16_t colors[] = {
      [CARDS_SOURCE_NONE] = UI_COLOR_GRAY,
      [CARDS_SOURCE_PDR] = UI_COLOR_YELLOW,
      [CARDS_SOURCE_GPS_PDR] = UI_COLOR_GREEN,
      [CARDS_SOURCE_GPS] = UI_COLOR_GREEN,
  };
  set_row(rows, 0, colors[c->source], "Src: %s", source_text(c->source));
  if (c->latlon_valid) {
    coordinate(buffer, sizeof(buffer), c->lat_e7, 'N', 'S');
    set_row(rows, 1, UI_COLOR_WHITE, "%s", buffer);
    coordinate(buffer, sizeof(buffer), c->lon_e7, 'E', 'W');
    set_row(rows, 2, UI_COLOR_WHITE, "%s", buffer);
    set_row(rows, 3, UI_COLOR_WHITE, "Err: %.1f m", (double)c->sigma_m);
  } else {
    set_row(rows, 1, UI_COLOR_GRAY, "No GPS anchor");
    set_row(rows, 2, UI_COLOR_GRAY, "(local only)");
    set_row(rows, 3, UI_COLOR_GRAY, "Err: ---");
  }
  if (!c->gps_on) {
    set_row(rows, 4, UI_COLOR_GRAY, "GPS: off");
  } else if (!c->gps_fix) {
    set_row(rows, 4, UI_COLOR_YELLOW, "GPS: no fix");
  } else {
    set_row(rows, 4, UI_COLOR_WHITE, "Sat %u HDOP %.1f", c->satellites,
            (double)c->hdop);
  }
  set_row(rows, 5, UI_COLOR_GRAY, "Fix %lu rej %lu", (unsigned long)c->gps_used,
          (unsigned long)c->gps_rejected);
  if (c->altitude_valid) {
    set_row(rows, 6, UI_COLOR_WHITE, "Alt: %.0f m %s", (double)c->altitude_m,
            c->altitude_gps_ref ? "" : "*");
  } else {
    set_row(rows, 6, UI_COLOR_GRAY, "Alt: ---");
  }
  distance_row(rows, 7, "Track:", c->track_m, UI_COLOR_WHITE);
  if (c->heading_valid) {
    bearing_row(rows, 8, "Hdg:", c->heading_deg);
  } else {
    set_row(rows, 8, UI_COLOR_GRAY, "Hdg: ---");
  }
  if (c->altitude_valid && !c->altitude_gps_ref) {
    set_row(rows, 9, UI_COLOR_GRAY, "* alt not GPS");
  }
}

void cards_alt(const cards_alt_t *c, ui_row_t *rows) {
  clear_rows(rows);
  set_row(rows, 0, trip_colors[c->state], "ALT: %s", trip_states[c->state]);
  if (!c->valid) {
    set_row(rows, 1, UI_COLOR_RED, "No barometer");
    return;
  }
  set_row(rows, 1, UI_COLOR_WHITE, "Alt: %.0f m", (double)c->altitude_m);
  set_row(rows, 2, UI_COLOR_WHITE, "Asc: +%.0f m", (double)c->ascent_m);
  set_row(rows, 3, UI_COLOR_WHITE, "Desc: -%.0f m", (double)c->descent_m);
  distance_row(rows, 4, "Horiz:", c->horizontal_m, UI_COLOR_WHITE);
  distance_row(rows, 5, "Slope:", c->slope_m, UI_COLOR_WHITE);
  set_row(rows, 6, UI_COLOR_WHITE, "P: %.1f hPa", (double)c->pressure_hpa);
  set_row(rows, 8, UI_COLOR_GRAY, "Step: %.0f m", (double)c->threshold_m);
  set_row(rows, 9, UI_COLOR_GRAY, c->gps_ref ? "Ref: GPS" : "Ref: std atm");
}

void cards_wpt(const cards_wpt_t *c, ui_row_t *rows) {
  clear_rows(rows);
  char buffer[24];
  if (!c->storage_ok) {
    set_row(rows, 0, UI_COLOR_RED, "Flash error");
    return;
  }
  if (c->selecting && c->clear_selected) {
    set_row(rows, 0, UI_COLOR_RED, "> CLEAR ALL");
    set_row(rows, 1, UI_COLOR_WHITE, "%u waypoints", c->count);
    set_row(rows, 3, UI_COLOR_GRAY, "OK: choose");
    set_row(rows, 4, UI_COLOR_GRAY, "BACK: cancel");
    return;
  }
  if (!c->has_item) {
    set_row(rows, 0, UI_COLOR_GRAY, "No waypoints");
    set_row(rows, 2, UI_COLOR_GRAY, "OK: mark here");
    set_row(rows, 9, UI_COLOR_GRAY, "Free: %u", c->free_slots);
    return;
  }
  set_row(rows, 0, c->selecting ? UI_COLOR_YELLOW : UI_COLOR_WHITE,
          "%sWP %u (%u)", c->selecting ? "> " : "", c->id, c->count);
  if (c->latlon_valid) {
    coordinate(buffer, sizeof(buffer), c->lat_e7, 'N', 'S');
    set_row(rows, 1, UI_COLOR_WHITE, "%s", buffer);
    coordinate(buffer, sizeof(buffer), c->lon_e7, 'E', 'W');
    set_row(rows, 2, UI_COLOR_WHITE, "%s", buffer);
  } else {
    set_row(rows, 1, UI_COLOR_GRAY, "No GPS pos");
  }
  if (c->altitude_valid) {
    set_row(rows, 3, UI_COLOR_WHITE, "Alt: %.0f m", (double)c->altitude_m);
  }
  if (c->relative_valid) {
    distance_row(rows, 4, "Dist:", c->distance_m, UI_COLOR_WHITE);
    bearing_row(rows, 5, "Brg:", c->bearing_deg);
    if (c->turn_valid) {
      turn_row(rows, 6, c->turn_deg);
    } else {
      set_row(rows, 6, UI_COLOR_GRAY, "Turn: ---");
    }
  } else {
    set_row(rows, 4, UI_COLOR_GRAY, "Dist: ---");
  }
  if (c->dalt_valid) {
    signed_m_row(rows, 7, "dAlt:", c->dalt_m);
  }
  if (c->time_valid) {
    set_row(rows, 8, UI_COLOR_GRAY, "At %02u:%02u %02u.%02u", c->hour,
            c->minute, c->day, c->month);
  }
  set_row(rows, 9, UI_COLOR_GRAY, "Free: %u", c->free_slots);
}

void cards_dist(const cards_dist_t *c, ui_row_t *rows) {
  clear_rows(rows);
  set_row(rows, 0, c->selecting == 1 ? UI_COLOR_YELLOW : UI_COLOR_WHITE,
          "%sA: %s", c->selecting == 1 ? ">" : "", c->a_label);
  set_row(rows, 1, c->selecting == 2 ? UI_COLOR_YELLOW : UI_COLOR_WHITE,
          "%sB: %s", c->selecting == 2 ? ">" : "", c->b_label);
  if (c->valid) {
    distance_row(rows, 2, "Horiz:", c->horizontal_m, UI_COLOR_WHITE);
    if (c->dalt_valid) {
      float slope = sqrtf(c->horizontal_m * c->horizontal_m +
                          c->dalt_m * c->dalt_m);
      distance_row(rows, 3, "Slope:", slope, UI_COLOR_WHITE);
      signed_m_row(rows, 4, "dAlt:", c->dalt_m);
      if (c->horizontal_m >= 1.0f) {
        set_row(rows, 5, UI_COLOR_WHITE, "Grade: %+.1f %%",
                (double)(100.0f * c->dalt_m / c->horizontal_m));
      }
    } else {
      set_row(rows, 3, UI_COLOR_GRAY, "Slope: ---");
    }
    if (c->track_valid) {
      distance_row(rows, 6, "Track:", c->track_m, UI_COLOR_WHITE);
    } else {
      set_row(rows, 6, UI_COLOR_GRAY, "Track: ---");
    }
    if (c->horizontal_m >= 1.0f) {
      bearing_row(rows, 7, "Brg:", c->bearing_deg);
    }
  } else {
    set_row(rows, 2, UI_COLOR_GRAY, "Horiz: ---");
  }
  if (c->guide != NULL) {
    set_row(rows, 9, UI_COLOR_GRAY, "%s", c->guide);
  }
}

void cards_line(const cards_line_t *c, ui_row_t *rows) {
  clear_rows(rows);
  if (!c->position_ok) {
    set_row(rows, 0, UI_COLOR_WHITE, "Line");
    set_row(rows, 2, UI_COLOR_RED, "Needs IMU+mag");
    return;
  }
  switch (c->stage) {
  case CARDS_LINE_SET_A:
    set_row(rows, 0, UI_COLOR_WHITE, "Line: set A");
    set_row(rows, 2, UI_COLOR_GRAY, "Stand at A,");
    set_row(rows, 3, UI_COLOR_GRAY, "press OK, walk");
    set_row(rows, 4, UI_COLOR_GRAY, "the line, OK");
    set_row(rows, 5, UI_COLOR_GRAY, "again at B.");
    return;
  case CARDS_LINE_SET_B:
    set_row(rows, 0, UI_COLOR_WHITE, "Line: walk to B");
    distance_row(rows, 2, "From A:", c->from_a_m,
                 c->from_a_m >= c->min_base_m ? UI_COLOR_GREEN
                                              : UI_COLOR_YELLOW);
    set_row(rows, 3, UI_COLOR_GRAY, "Min: %.0f m", (double)c->min_base_m);
    set_row(rows, 5, UI_COLOR_GRAY, "Hold device");
    set_row(rows, 6, UI_COLOR_GRAY, "forward");
    return;
  case CARDS_LINE_ACTIVE:
  default:
    break;
  }
  float turn = 0.0f;
  bool reverse = false;
  if (c->heading_valid) {
    turn = fmodf(c->bearing_deg - c->heading_deg + 540.0f, 360.0f) - 180.0f;
    if (fabsf(turn) > 90.0f) {
      reverse = true;
      turn = fmodf(turn + 360.0f, 360.0f) - 180.0f;
    }
  }
  float offset = reverse ? -c->offset_m : c->offset_m;
  set_row(rows, 0, UI_COLOR_WHITE, "Line: %u %s",
          ui_round_degrees(c->bearing_deg), ui_cardinal(c->bearing_deg));
  float off = fabsf(offset);
  uint16_t color = (off < 1.0f)   ? UI_COLOR_GREEN
                   : (off < 3.0f) ? UI_COLOR_YELLOW
                                  : UI_COLOR_RED;
  if (off < 0.05f) {
    set_row(rows, 1, color, "Off: 0.0 m");
  } else {
    char value[16];
    if (off < 99.95f) {
      snprintf(value, sizeof(value), "%.1f m", (double)off);
    } else {
      ui_format_distance(value, sizeof(value), off);
    }
    set_row(rows, 1, color, "Off: %c %s", (offset > 0.0f) ? 'R' : 'L', value);
    set_row(rows, 2, UI_COLOR_GRAY, "Go %s", (offset > 0.0f) ? "left" : "right");
  }
  set_row(rows, 3, UI_COLOR_WHITE, "Along: %.1f m", (double)c->along_m);
  if (c->heading_valid) {
    bearing_row(rows, 4, "Hdg:", c->heading_deg);
    turn_row(rows, 5, turn);
  } else {
    set_row(rows, 4, UI_COLOR_GRAY, "Hdg: ---");
  }
  set_row(rows, 6, UI_COLOR_GRAY, "AB: %.1f m", (double)c->base_m);
  if (reverse) {
    set_row(rows, 7, UI_COLOR_GRAY, "Facing A");
  }
  set_row(rows, 9, UI_COLOR_GRAY, "Src: steps");
}

static void area_value(char *buffer, size_t size, float m2) {
  if (m2 < 9999.5f) {
    snprintf(buffer, size, "%.0f m2", (double)m2);
  } else if (m2 < 999950.0f) {
    snprintf(buffer, size, "%.3f ha", (double)(m2 / 10000.0f));
  } else {
    snprintf(buffer, size, "%.3f km2", (double)(m2 / 1e6f));
  }
}

void cards_area(const cards_area_t *c, ui_row_t *rows) {
  clear_rows(rows);
  char value[16];
  set_row(rows, 0, UI_COLOR_WHITE, "Points: %u/%u", c->count, c->max);
  if (c->count >= 3) {
    area_value(value, sizeof(value), c->area_m2);
    set_row(rows, 1, c->crossing ? UI_COLOR_RED : UI_COLOR_GREEN, "Area: %s",
            value);
    distance_row(rows, 2, "Perim:", c->perimeter_m, UI_COLOR_WHITE);
  } else {
    set_row(rows, 1, UI_COLOR_GRAY, "Area: ---");
    set_row(rows, 2, UI_COLOR_GRAY, "Need 3 points");
  }
  if (c->last_valid) {
    distance_row(rows, 3, "Edge:", c->last_edge_m, UI_COLOR_WHITE);
  }
  if (c->here_valid) {
    distance_row(rows, 4, "To 1st:", c->to_first_m, UI_COLOR_WHITE);
    if (c->count >= 2) {
      area_value(value, sizeof(value), c->area_with_here_m2);
      set_row(rows, 5, UI_COLOR_GRAY, "+here: %s", value);
    }
  }
  if (c->crossing) {
    set_row(rows, 6, UI_COLOR_RED, "Edges cross!");
  }
  if (c->count >= c->max) {
    set_row(rows, 7, UI_COLOR_YELLOW, "Full");
  }
  set_row(rows, 9, UI_COLOR_GRAY, "Src: %s", source_text(c->source));
}

static void hm_row(ui_row_t *rows, uint8_t row, const char *label,
                   uint16_t minutes) {
  set_row(rows, row, UI_COLOR_WHITE, "%s %02u:%02u", label, minutes / 60u,
          minutes % 60u);
}

void cards_sun(const cards_sun_t *c, ui_row_t *rows) {
  clear_rows(rows);
  if (!c->time_valid) {
    set_row(rows, 0, UI_COLOR_RED, "No valid time");
    return;
  }
  if (!c->position_valid) {
    set_row(rows, 0, UI_COLOR_YELLOW, "No position");
    set_row(rows, 2, UI_COLOR_GRAY, "Needs a GPS fix");
    set_row(rows, 3, UI_COLOR_GRAY, "or a waypoint");
    return;
  }
  switch (c->status) {
  case CARDS_SUN_ALWAYS_UP:
    set_row(rows, 0, UI_COLOR_YELLOW, "Polar day");
    break;
  case CARDS_SUN_ALWAYS_DOWN:
    set_row(rows, 0, UI_COLOR_YELLOW, "Polar night");
    break;
  default:
    hm_row(rows, 0, "Rise:", c->rise_min);
    hm_row(rows, 1, "Set: ", c->set_min);
    set_row(rows, 3, UI_COLOR_WHITE, "Day: %u:%02u h", c->day_length_min / 60u,
            c->day_length_min % 60u);
    break;
  }
  hm_row(rows, 2, "Noon:", c->noon_min);
  if (c->countdown_valid) {
    set_row(rows, 4, c->to_set ? UI_COLOR_YELLOW : UI_COLOR_WHITE,
            "To %s %lu:%02lu", c->to_set ? "set: " : "rise:",
            (unsigned long)(c->countdown_min / 60u),
            (unsigned long)(c->countdown_min % 60u));
  }
  if (c->civil_status == CARDS_SUN_OK) {
    hm_row(rows, 5, "Dawn:", c->dawn_min);
    hm_row(rows, 6, "Dusk:", c->dusk_min);
  }
  set_row(rows, 7, UI_COLOR_WHITE, "Elev: %.1f", (double)c->elevation_deg);
  bearing_row(rows, 8, "Az:", c->azimuth_deg);
  set_row(rows, 9, UI_COLOR_GRAY, "Pos: %s", c->position_source);
}
