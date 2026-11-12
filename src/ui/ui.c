#include "ui.h"
#include "board_config.h"
#include "datetime_utils.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEG "\x7f"

static uint8_t shift_x = 0;
static uint8_t shift_y = 0;

void ui_set_shift(uint8_t dx, uint8_t dy) {
  shift_x = (dx > UI_SHIFT_MAX_X) ? UI_SHIFT_MAX_X : dx;
  shift_y = (dy > UI_SHIFT_MAX_Y) ? UI_SHIFT_MAX_Y : dy;
}

#define COMPASS_TEXT_COLUMNS 9
#define COMPASS_CX (UI_TEXT_WIDTH - 24)
#define COMPASS_CY 26
#define COMPASS_RADIUS 23
#define COMPASS_NEEDLE_LENGTH 19
#define COMPASS_TICK_LENGTH 3
#define COMPASS_HUB_SIZE 3
#define DEG_TO_RAD (3.14159265358979f / 180.0f)

typedef struct {
  bool drawn;
  uint16_t color;
  int16_t north_x;
  int16_t north_y;
  int16_t south_x;
  int16_t south_y;
} needle_t;

static needle_t needle;

void ui_clear(ssd1351_t *oled) {
  ssd1351_fill(oled, UI_COLOR_BLACK);
  needle.drawn = false;
}

static uint8_t row_y(uint8_t row) {
  return (uint8_t)(shift_y + row * UI_ROW_HEIGHT);
}

static void draw_padded(ssd1351_t *oled, uint8_t column, uint8_t row,
                        const char *text, uint8_t width, uint16_t color) {
  char line[UI_COLUMNS + 1];
  size_t length = strlen(text);
  if (length > width) {
    length = width;
  }
  memcpy(line, text, length);
  memset(line + length, ' ', width - length);
  line[width] = '\0';
  ssd1351_draw_string(oled, (uint8_t)(shift_x + column * UI_CHAR_WIDTH),
                      row_y(row), line, color, UI_COLOR_BLACK);
}

void ui_draw_line(ssd1351_t *oled, uint8_t row, const char *text,
                  uint16_t color) {
  draw_padded(oled, 0, row, text, UI_COLUMNS, color);
}

void ui_draw_centered(ssd1351_t *oled, uint8_t row, const char *text,
                      uint16_t color) {
  size_t length = strlen(text);
  if (length > UI_COLUMNS) {
    length = UI_COLUMNS;
  }
  uint8_t text_width = (uint8_t)(length * UI_CHAR_WIDTH);
  uint8_t left = (uint8_t)((UI_TEXT_WIDTH - text_width) / 2u);
  uint8_t right = (uint8_t)(UI_TEXT_WIDTH - text_width - left);
  uint8_t y = row_y(row);
  if (left > 0) {
    ssd1351_fill_rect(oled, shift_x, y, left, UI_CHAR_HEIGHT, UI_COLOR_BLACK);
  }
  if (right > 0) {
    ssd1351_fill_rect(oled, (uint8_t)(shift_x + left + text_width), y, right,
                      UI_CHAR_HEIGHT, UI_COLOR_BLACK);
  }
  char buffer[UI_COLUMNS + 1];
  memcpy(buffer, text, length);
  buffer[length] = '\0';
  ssd1351_draw_string(oled, (uint8_t)(shift_x + left), y, buffer, color,
                      UI_COLOR_BLACK);
}

void ui_draw_status(ssd1351_t *oled, const char *text, uint16_t color) {
  ui_draw_line(oled, UI_STATUS_ROW, text, color);
}

void ui_draw_footer(ssd1351_t *oled, const char *title, uint8_t index,
                    uint8_t count, const ui_tag_t *tags, uint8_t tag_count) {
  char buffer[32];
  uint8_t tags_width = 0;
  for (uint8_t i = 0; i < tag_count; i++) {
    tags_width = (uint8_t)(tags_width + 1u + strlen(tags[i].text));
  }
  if (tags_width > UI_COLUMNS) {
    tags_width = UI_COLUMNS;
  }
  uint8_t title_width = (uint8_t)(UI_COLUMNS - tags_width);

  snprintf(buffer, sizeof(buffer), "%u/%u %s", index, count, title);
  if (strlen(buffer) > title_width) {
    snprintf(buffer, sizeof(buffer), "%u %s", index, title);
  }
  if (strlen(buffer) > title_width) {
    snprintf(buffer, sizeof(buffer), "%s", title);
  }
  draw_padded(oled, 0, UI_FOOTER_ROW, buffer, title_width, UI_COLOR_GRAY);
  uint8_t column = title_width;
  for (uint8_t i = 0; i < tag_count && column < UI_COLUMNS; i++) {
    uint8_t width = (uint8_t)(1u + strlen(tags[i].text));
    if (column + width > UI_COLUMNS) {
      width = (uint8_t)(UI_COLUMNS - column);
    }
    snprintf(buffer, sizeof(buffer), " %s", tags[i].text);
    draw_padded(oled, column, UI_FOOTER_ROW, buffer, width, tags[i].color);
    column = (uint8_t)(column + width);
  }
}

static void clear_rows(ssd1351_t *oled, uint8_t first, uint8_t last) {
  for (uint8_t row = first; row <= last; row++) {
    ui_draw_line(oled, row, "", UI_COLOR_WHITE);
  }
}

static void format_date(char *buffer, size_t size, uint8_t day, uint8_t month,
                        uint8_t year) {
  snprintf(buffer, size, "%02u.%02u.%04u", day, month, CENTURY_BASE + year);
}

void ui_draw_environment_card(ssd1351_t *oled, const ui_environment_t *env,
                              const ds3231_datetime_t *utc,
                              bool rtc_time_valid) {
  char buffer[32];
  if (env->climate_valid) {
    snprintf(buffer, sizeof(buffer), "T: %.1f " DEG "C",
             (double)env->temperature);
    ui_draw_line(oled, 0, buffer, UI_COLOR_WHITE);
    snprintf(buffer, sizeof(buffer), "H: %.1f %%", (double)env->humidity);
    ui_draw_line(oled, 1, buffer, UI_COLOR_WHITE);
  } else {
    ui_draw_line(oled, 0, "T: ---", UI_COLOR_RED);
    ui_draw_line(oled, 1, "H: ---", UI_COLOR_RED);
  }
  if (env->pressure_valid) {
    snprintf(buffer, sizeof(buffer), "P: %.1f hPa",
             (double)(env->pressure / 100.0f));
    ui_draw_line(oled, 2, buffer, UI_COLOR_WHITE);
  } else {
    ui_draw_line(oled, 2, "P: ---", UI_COLOR_RED);
  }
  ui_draw_line(oled, 3, "", UI_COLOR_WHITE);

  if (utc != NULL) {
    ds3231_datetime_t local;
    datetime_to_local(utc, &local, CENTURY_BASE);
    format_date(buffer, sizeof(buffer), local.day, local.month, local.year);
    ui_draw_centered(oled, 4, buffer, UI_COLOR_WHITE);
    snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u", local.hours,
             local.minutes, local.seconds);
    ui_draw_centered(oled, 6, buffer, UI_COLOR_WHITE);
  } else {
    ui_draw_centered(oled, 4, "--.--.----", UI_COLOR_RED);
    ui_draw_centered(oled, 6, "--:--:--", UI_COLOR_RED);
  }
  ui_draw_line(oled, 5, "", UI_COLOR_WHITE);
  ui_draw_line(oled, 7, rtc_time_valid ? "" : "RTC invalid", UI_COLOR_RED);
  clear_rows(oled, 8, UI_CONTENT_ROWS - 1);
}

static const char *gps_state_text(ui_gps_state_t state, uint16_t *color) {
  switch (state) {
  case UI_GPS_OFF:
    *color = UI_COLOR_GRAY;
    return "GPS: OFF";
  case UI_GPS_STARTING:
    *color = UI_COLOR_YELLOW;
    return "GPS: STARTING";
  case UI_GPS_FIX:
    *color = UI_COLOR_GREEN;
    return "GPS: FIX";
  case UI_GPS_LOST:
    *color = UI_COLOR_YELLOW;
    return "GPS: LOST";
  case UI_GPS_SEARCHING:
    *color = UI_COLOR_YELLOW;
    return "GPS: SEARCHING";
  case UI_GPS_IDLE:
    *color = UI_COLOR_YELLOW;
    return "GPS: IDLE";
  case UI_GPS_TIMEOUT:
    *color = UI_COLOR_RED;
    return "GPS: TIMEOUT";
  case UI_GPS_NO_DATA:
  default:
    *color = UI_COLOR_RED;
    return "GPS: NO DATA";
  }
}

static bool gps_is_receiving(ui_gps_state_t state) {
  return state == UI_GPS_SEARCHING || state == UI_GPS_LOST ||
         state == UI_GPS_FIX;
}

static void format_age(char *buffer, size_t size, uint32_t seconds) {
  unsigned long s = (unsigned long)seconds;
  if (s < 60u) {
    snprintf(buffer, size, "%lus", s);
  } else if (s < 3600u) {
    snprintf(buffer, size, "%lum%02lus", s / 60u, s % 60u);
  } else if (s < 86400u) {
    snprintf(buffer, size, "%luh%02lum", s / 3600u, (s % 3600u) / 60u);
  } else {
    snprintf(buffer, size, "%lud%02luh", s / 86400u, (s % 86400u) / 3600u);
  }
}

static void draw_state_line(ssd1351_t *oled, const ui_gps_t *gps) {
  uint16_t color;
  const char *state = gps_state_text(gps->state, &color);
  ui_draw_line(oled, 0, state, color);
}

static void format_coordinate(char *buffer, size_t size, int32_t value_e7,
                              char positive, char negative) {
  uint32_t magnitude =
      (value_e7 < 0) ? (uint32_t)(-(int64_t)value_e7) : (uint32_t)value_e7;
  snprintf(buffer, size, "%c: %lu.%05lu", (value_e7 < 0) ? negative : positive,
           (unsigned long)(magnitude / 10000000u),
           (unsigned long)((magnitude % 10000000u) / 100u));
}

void ui_draw_position_card(ssd1351_t *oled, const ui_gps_t *gps) {
  char buffer[32];
  const neo6m_data_t *f = &gps->fix;
  uint16_t color = (gps->state == UI_GPS_FIX) ? UI_COLOR_WHITE : UI_COLOR_GRAY;
  draw_state_line(oled, gps);

  if (gps->has_fix && f->altitude_valid) {
    int32_t alt = f->altitude_dm;
    uint32_t magnitude = (alt < 0) ? (uint32_t)(-(int64_t)alt) : (uint32_t)alt;
    snprintf(buffer, sizeof(buffer), "Alt: %s%lu.%lu m", (alt < 0) ? "-" : "",
             (unsigned long)(magnitude / 10u),
             (unsigned long)(magnitude % 10u));
    ui_draw_line(oled, 1, buffer, color);
  } else {
    ui_draw_line(oled, 1, "Alt: ---", color);
  }

  if (gps->has_fix && f->speed_valid) {
    snprintf(buffer, sizeof(buffer), "Spd: %lu.%lu km/h",
             (unsigned long)(f->speed_kmh_x10 / 10u),
             (unsigned long)(f->speed_kmh_x10 % 10u));
    ui_draw_line(oled, 2, buffer, color);
  } else {
    ui_draw_line(oled, 2, "Spd: ---", color);
  }

  if (gps->has_fix && f->course_valid) {
    snprintf(buffer, sizeof(buffer), "Crs: %u.%u " DEG, f->course_x100 / 100u,
             (f->course_x100 % 100u) / 10u);
    ui_draw_line(oled, 3, buffer, color);
  } else {
    ui_draw_line(oled, 3, "Crs: ---", color);
  }

  if (gps->has_fix && gps->state != UI_GPS_FIX) {
    char age[16];
    if (gps->fix_age_valid) {
      format_age(age, sizeof(age), gps->fix_age_s);
    } else {
      snprintf(age, sizeof(age), "?");
    }
    snprintf(buffer, sizeof(buffer), "Age: %s", age);
    ui_draw_line(oled, 4, buffer, UI_COLOR_GRAY);
  } else {
    ui_draw_line(oled, 4, "", UI_COLOR_GRAY);
  }
  ui_draw_line(oled, 5, "", UI_COLOR_WHITE);

  if (gps->has_fix && f->position_valid) {
    format_coordinate(buffer, sizeof(buffer), f->latitude_e7, 'N', 'S');
    ui_draw_line(oled, 6, buffer, color);
    format_coordinate(buffer, sizeof(buffer), f->longitude_e7, 'E', 'W');
    ui_draw_line(oled, 7, buffer, color);
  } else {
    ui_draw_line(oled, 6, "Lat: ---", color);
    ui_draw_line(oled, 7, "Lon: ---", color);
  }
  clear_rows(oled, 8, UI_CONTENT_ROWS - 1);
}

void ui_draw_satellite_card(ssd1351_t *oled, const ui_gps_t *gps) {
  char buffer[32];
  const neo6m_data_t *live = &gps->live;
  uint16_t live_color =
      gps_is_receiving(gps->state) ? UI_COLOR_WHITE : UI_COLOR_GRAY;
  uint16_t fix_color =
      (gps->state == UI_GPS_FIX) ? UI_COLOR_WHITE : UI_COLOR_GRAY;
  draw_state_line(oled, gps);

  if (live->sky_valid) {
    snprintf(buffer, sizeof(buffer), "Sat: %u/%u", live->satellites,
             live->satellites_in_view);
  } else {
    snprintf(buffer, sizeof(buffer), "Sat: %u", live->satellites);
  }
  ui_draw_line(oled, 1, buffer, live_color);

  if (live->sky_valid && live->cno_top4_avg > 0) {
    snprintf(buffer, sizeof(buffer), "CN0: %u dBHz", live->cno_top4_avg);
  } else {
    snprintf(buffer, sizeof(buffer), "CN0: ---");
  }
  ui_draw_line(oled, 2, buffer, live_color);

  const neo6m_data_t *f = &gps->fix;
  if (gps->has_fix && f->hdop_valid) {
    snprintf(buffer, sizeof(buffer), "HDOP: %u.%02u", f->hdop_x100 / 100u,
             f->hdop_x100 % 100u);
  } else {
    snprintf(buffer, sizeof(buffer), "HDOP: ---");
  }
  ui_draw_line(oled, 3, buffer, fix_color);
  ui_draw_line(oled, 4, "", UI_COLOR_WHITE);

  if (live->time_valid) {
    snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u UTC", live->hours,
             live->minutes, live->seconds);
  } else {
    snprintf(buffer, sizeof(buffer), "--:--:-- UTC");
  }
  ui_draw_centered(oled, 5, buffer, live_color);
  ui_draw_line(oled, 6, "", UI_COLOR_WHITE);

  if (live->date_valid) {
    format_date(buffer, sizeof(buffer), live->day, live->month, live->year);
  } else {
    snprintf(buffer, sizeof(buffer), "--.--.----");
  }
  ui_draw_centered(oled, 7, buffer, live_color);

  unsigned long errors =
      (unsigned long)(gps->stats.checksum_errors + gps->stats.overflows +
                      gps->dropped_bytes);
  snprintf(buffer, sizeof(buffer), "Rx: %lu",
           (unsigned long)gps->stats.sentences);
  ui_draw_line(oled, 8, buffer, UI_COLOR_GRAY);
  snprintf(buffer, sizeof(buffer), "Err: %lu", errors);
  ui_draw_line(oled, 9, buffer, UI_COLOR_GRAY);
}

// ---------------------------------------------------------------------------
// Compass card
// ---------------------------------------------------------------------------

static void plot(ssd1351_t *oled, int16_t x, int16_t y, uint16_t color) {
  int16_t sx = (int16_t)(x + shift_x);
  int16_t sy = (int16_t)(y + shift_y);
  if (sx < 0 || sy < 0 || sx >= UI_SCREEN_SIZE || sy >= UI_SCREEN_SIZE) {
    return;
  }
  ssd1351_draw_pixel(oled, (uint8_t)sx, (uint8_t)sy, color);
}

static void draw_segment(ssd1351_t *oled, int16_t x0, int16_t y0, int16_t x1,
                         int16_t y1, uint16_t color) {
  int16_t dx = (int16_t)((x1 > x0) ? x1 - x0 : x0 - x1);
  int16_t dy = (int16_t)(-((y1 > y0) ? y1 - y0 : y0 - y1));
  int16_t step_x = (x0 < x1) ? 1 : -1;
  int16_t step_y = (y0 < y1) ? 1 : -1;
  int16_t error = (int16_t)(dx + dy);
  while (true) {
    plot(oled, x0, y0, color);
    if (x0 == x1 && y0 == y1) {
      return;
    }
    int16_t doubled = (int16_t)(2 * error);
    if (doubled >= dy) {
      error = (int16_t)(error + dy);
      x0 = (int16_t)(x0 + step_x);
    }
    if (doubled <= dx) {
      error = (int16_t)(error + dx);
      y0 = (int16_t)(y0 + step_y);
    }
  }
}

static void draw_circle(ssd1351_t *oled, int16_t cx, int16_t cy, int16_t r,
                        uint16_t color) {
  int16_t x = r;
  int16_t y = 0;
  int16_t error = (int16_t)(1 - r);
  while (x >= y) {
    plot(oled, (int16_t)(cx + x), (int16_t)(cy + y), color);
    plot(oled, (int16_t)(cx + y), (int16_t)(cy + x), color);
    plot(oled, (int16_t)(cx - y), (int16_t)(cy + x), color);
    plot(oled, (int16_t)(cx - x), (int16_t)(cy + y), color);
    plot(oled, (int16_t)(cx - x), (int16_t)(cy - y), color);
    plot(oled, (int16_t)(cx - y), (int16_t)(cy - x), color);
    plot(oled, (int16_t)(cx + y), (int16_t)(cy - x), color);
    plot(oled, (int16_t)(cx + x), (int16_t)(cy - y), color);
    y++;
    if (error < 0) {
      error = (int16_t)(error + 2 * y + 1);
    } else {
      x--;
      error = (int16_t)(error + 2 * (y - x) + 1);
    }
  }
}

static void erase_needle(ssd1351_t *oled) {
  if (!needle.drawn) {
    return;
  }
  draw_segment(oled, COMPASS_CX, COMPASS_CY, needle.north_x, needle.north_y,
               UI_COLOR_BLACK);
  draw_segment(oled, COMPASS_CX, COMPASS_CY, needle.south_x, needle.south_y,
               UI_COLOR_BLACK);
  needle.drawn = false;
}

static void update_needle(ssd1351_t *oled, bool show, float heading_deg,
                          uint16_t color) {
  needle_t next = {0};
  if (show) {
    float angle = heading_deg * DEG_TO_RAD;
    int16_t ox = (int16_t)lroundf((float)COMPASS_NEEDLE_LENGTH * sinf(angle));
    int16_t oy = (int16_t)lroundf((float)COMPASS_NEEDLE_LENGTH * cosf(angle));
    next.drawn = true;
    next.color = color;
    next.north_x = (int16_t)(COMPASS_CX - ox);
    next.north_y = (int16_t)(COMPASS_CY - oy);
    next.south_x = (int16_t)(COMPASS_CX + ox);
    next.south_y = (int16_t)(COMPASS_CY + oy);
  }
  if (needle.drawn && next.drawn && needle.color == next.color &&
      needle.north_x == next.north_x && needle.north_y == next.north_y) {
    return;
  }
  erase_needle(oled);
  if (!next.drawn) {
    return;
  }
  draw_segment(oled, COMPASS_CX, COMPASS_CY, next.south_x, next.south_y,
               UI_COLOR_GRAY);
  draw_segment(oled, COMPASS_CX, COMPASS_CY, next.north_x, next.north_y,
               next.color);
  needle = next;
}

static void draw_rose(ssd1351_t *oled) {
  draw_circle(oled, COMPASS_CX, COMPASS_CY, COMPASS_RADIUS, UI_COLOR_GRAY);
  int16_t top = (int16_t)(COMPASS_CY - COMPASS_RADIUS - COMPASS_TICK_LENGTH);
  draw_segment(oled, COMPASS_CX, top, COMPASS_CX,
               (int16_t)(COMPASS_CY - COMPASS_RADIUS - 1), UI_COLOR_WHITE);
}

static void draw_hub(ssd1351_t *oled) {
  int16_t x = (int16_t)(shift_x + COMPASS_CX - COMPASS_HUB_SIZE / 2);
  int16_t y = (int16_t)(shift_y + COMPASS_CY - COMPASS_HUB_SIZE / 2);
  ssd1351_fill_rect(oled, (uint8_t)x, (uint8_t)y, COMPASS_HUB_SIZE,
                    COMPASS_HUB_SIZE, UI_COLOR_WHITE);
}

static unsigned rounded_degrees(float degrees) {
  long value = lroundf(degrees);
  value %= 360;
  if (value < 0) {
    value += 360;
  }
  return (unsigned)value;
}

static const char *cardinal(float degrees) {
  static const char *const names[] = {"N", "NE", "E", "SE",
                                      "S", "SW", "W", "NW"};
  unsigned sector = (rounded_degrees(degrees) * 2u + 45u) / 90u;
  return names[sector % 8u];
}

static void format_ut(char *buffer, size_t size, float value) {
  if (value > -99.95f && value < 99.95f) {
    snprintf(buffer, size, "%.1f", (double)value);
  } else {
    snprintf(buffer, size, "%.0f", (double)value);
  }
}

static void draw_compass_text(ssd1351_t *oled, uint8_t row, const char *text,
                              uint16_t color) {
  draw_padded(oled, 0, row, text, COMPASS_TEXT_COLUMNS, color);
}

static void draw_compass_headings(ssd1351_t *oled, const ui_compass_t *c,
                                  uint16_t color) {
  char buffer[32];
  if (c->heading_valid) {
    snprintf(buffer, sizeof(buffer), "Hdg: %u" DEG,
             rounded_degrees(c->heading_true_deg));
    draw_compass_text(oled, 0, buffer, color);
    snprintf(buffer, sizeof(buffer), "Mag: %u" DEG,
             rounded_degrees(c->heading_magnetic_deg));
    draw_compass_text(oled, 1, buffer, color);
    snprintf(buffer, sizeof(buffer), "Dir: %s", cardinal(c->heading_true_deg));
    draw_compass_text(oled, 2, buffer, color);
  } else {
    draw_compass_text(oled, 0, "Hdg: ---", color);
    draw_compass_text(oled, 1, "Mag: ---", color);
    draw_compass_text(oled, 2, "Dir: ---", color);
  }
  static const char *const reference_names[] = {
      [UI_COMPASS_REF_LEVEL] = "Ref: lvl",
      [UI_COMPASS_REF_TOP] = "Ref: top",
      [UI_COMPASS_REF_BACK] = "Ref: back",
  };
  draw_compass_text(oled, 3, reference_names[c->reference],
                    (c->reference == UI_COMPASS_REF_LEVEL) ? UI_COLOR_GRAY
                                                           : UI_COLOR_WHITE);
  snprintf(buffer, sizeof(buffer),
           (c->declination_deg < 0.0f) ? "Dec:%.1f" DEG : "Dec: %.1f" DEG,
           (double)c->declination_deg);
  draw_compass_text(oled, 4, buffer, UI_COLOR_GRAY);
}

static void draw_compass_parameters(ssd1351_t *oled, const ui_compass_t *c,
                                    uint16_t color) {
  char buffer[32];
  char a[12];
  char b[12];
  if (!c->data_valid) {
    clear_rows(oled, 6, UI_CONTENT_ROWS - 1);
    ui_draw_line(oled, 6, "B: ---", UI_COLOR_GRAY);
    return;
  }
  if (c->overflow) {
    ui_draw_line(oled, 6, "B: overflow", UI_COLOR_RED);
  } else {
    format_ut(a, sizeof(a), c->field_ut);
    snprintf(buffer, sizeof(buffer), "B: %s uT", a);
    ui_draw_line(oled, 6, buffer,
                 c->vertical_calibrated ? color : UI_COLOR_GRAY);
  }
  if (c->vertical_calibrated && !c->overflow) {
    snprintf(buffer, sizeof(buffer), "Inc: %.1f" DEG,
             (double)c->inclination_deg);
    ui_draw_line(oled, 7, buffer, color);
  } else {
    ui_draw_line(oled, 7, "Inc: ---", UI_COLOR_GRAY);
  }
  format_ut(a, sizeof(a), c->x_ut);
  format_ut(b, sizeof(b), c->y_ut);
  snprintf(buffer, sizeof(buffer), "X:%s Y:%s", a, b);
  ui_draw_line(oled, 8, buffer, UI_COLOR_GRAY);
  format_ut(a, sizeof(a), c->z_ut);
  snprintf(buffer, sizeof(buffer), "Z:%s uT", a);
  ui_draw_line(oled, 9, buffer, UI_COLOR_GRAY);
}

static void draw_calibration_progress(ssd1351_t *oled, const ui_compass_t *c) {
  char buffer[32];
  ui_draw_line(oled, 6, "Turn 360, flip", UI_COLOR_YELLOW);
  snprintf(buffer, sizeof(buffer), "Rx:%3.0f Ry:%3.0f", (double)c->range_x_ut,
           (double)c->range_y_ut);
  ui_draw_line(oled, 7, buffer, UI_COLOR_WHITE);
  snprintf(buffer, sizeof(buffer), "Rz:%3.0f uT", (double)c->range_z_ut);
  ui_draw_line(oled, 8, buffer, UI_COLOR_WHITE);
  snprintf(buffer, sizeof(buffer), "Samples: %lu",
           (unsigned long)c->cal_samples);
  ui_draw_line(oled, 9, buffer, UI_COLOR_GRAY);
}

void ui_draw_compass_card(ssd1351_t *oled, const ui_compass_t *compass) {
  draw_rose(oled);
  ui_draw_line(oled, 5, "", UI_COLOR_WHITE);

  if (!compass->sensor_ok) {
    draw_compass_text(oled, 0, "Mag: ERR", UI_COLOR_RED);
    for (uint8_t row = 1; row <= 4; row++) {
      draw_compass_text(oled, row, "", UI_COLOR_WHITE);
    }
    clear_rows(oled, 6, UI_CONTENT_ROWS - 1);
    ui_draw_line(oled, 6, "No magnetometer", UI_COLOR_RED);
    update_needle(oled, false, 0.0f, UI_COLOR_BLACK);
    draw_hub(oled);
    return;
  }

  uint16_t color = compass->calibrated ? UI_COLOR_WHITE : UI_COLOR_GRAY;
  draw_compass_headings(oled, compass, color);
  update_needle(oled, compass->heading_valid && !compass->overflow,
                compass->heading_true_deg,
                compass->calibrated ? UI_COLOR_RED : UI_COLOR_YELLOW);
  draw_hub(oled);

  if (compass->calibrating) {
    draw_calibration_progress(oled, compass);
  } else {
    draw_compass_parameters(oled, compass, color);
  }
}

// ---------------------------------------------------------------------------
// IMU card
// ---------------------------------------------------------------------------

void ui_draw_imu_card(ssd1351_t *oled, const ui_imu_t *imu) {
  static const char axis_names[3] = {'X', 'Y', 'Z'};
  char buffer[32];
  if (!imu->sensor_ok) {
    clear_rows(oled, 0, UI_CONTENT_ROWS - 1);
    ui_draw_line(oled, 0, "IMU: ERR", UI_COLOR_RED);
    ui_draw_line(oled, 2, "No IMU", UI_COLOR_RED);
    return;
  }
  if (!imu->data_valid) {
    clear_rows(oled, 0, UI_CONTENT_ROWS - 1);
    ui_draw_line(oled, 0, "IMU: ---", UI_COLOR_GRAY);
    return;
  }
  for (uint8_t axis = 0; axis < 3; axis++) {
    snprintf(buffer, sizeof(buffer), "a%c: %+.2f g", axis_names[axis],
             (double)imu->accel_g[axis]);
    ui_draw_line(oled, axis, buffer, UI_COLOR_WHITE);
  }
  snprintf(buffer, sizeof(buffer), "|a|: %.2f g", (double)imu->accel_norm_g);
  ui_draw_line(oled, 3, buffer, UI_COLOR_GRAY);
  for (uint8_t axis = 0; axis < 3; axis++) {
    snprintf(buffer, sizeof(buffer), "g%c:%+7.1f" DEG "/s", axis_names[axis],
             (double)imu->gyro_dps[axis]);
    ui_draw_line(oled, (uint8_t)(4 + axis), buffer, UI_COLOR_WHITE);
  }
  if (imu->axes_ok) {
    snprintf(buffer, sizeof(buffer), "Pitch: %+.1f" DEG,
             (double)imu->pitch_deg);
    ui_draw_line(oled, 7, buffer, UI_COLOR_WHITE);
    snprintf(buffer, sizeof(buffer), "Roll: %+.1f" DEG, (double)imu->roll_deg);
    ui_draw_line(oled, 8, buffer, UI_COLOR_WHITE);
  } else {
    ui_draw_line(oled, 7, "Axes: bad cfg", UI_COLOR_RED);
    ui_draw_line(oled, 8, "", UI_COLOR_WHITE);
  }
  snprintf(buffer, sizeof(buffer), "Chip: %.1f" DEG "C",
           (double)imu->chip_temp_c);
  ui_draw_line(oled, 9, buffer, UI_COLOR_GRAY);
}

// ---------------------------------------------------------------------------
// Trip card
// ---------------------------------------------------------------------------

static void format_distance(char *buffer, size_t size, float metres) {
  if (metres < 999.5f) {
    snprintf(buffer, size, "%.0f m", (double)metres);
  } else if (metres < 99999.5f) {
    snprintf(buffer, size, "%.3f km", (double)(metres / 1000.0f));
  } else if (metres < 999950.0f) {
    snprintf(buffer, size, "%.2f km", (double)(metres / 1000.0f));
  } else {
    snprintf(buffer, size, "%.1f km", (double)(metres / 1000.0f));
  }
}

void ui_draw_trip_card(ssd1351_t *oled, const ui_trip_t *trip) {
  static const char *const state_names[] = {
      [UI_TRIP_STOPPED] = "Trip: STOP",
      [UI_TRIP_RUNNING] = "Trip: RUN",
      [UI_TRIP_PAUSED] = "Trip: PAUSE",
  };
  static const uint16_t state_colors[] = {
      [UI_TRIP_STOPPED] = UI_COLOR_GRAY,
      [UI_TRIP_RUNNING] = UI_COLOR_GREEN,
      [UI_TRIP_PAUSED] = UI_COLOR_YELLOW,
  };
  char buffer[32];
  char value[16];
  ui_draw_line(oled, 0, state_names[trip->state], state_colors[trip->state]);

  uint32_t s = trip->elapsed_s;
  snprintf(buffer, sizeof(buffer), "Time: %lu:%02lu:%02lu",
           (unsigned long)(s / 3600u), (unsigned long)(s / 60u % 60u),
           (unsigned long)(s % 60u));
  ui_draw_line(oled, 1, buffer, UI_COLOR_WHITE);
  snprintf(buffer, sizeof(buffer), "Steps: %lu", (unsigned long)trip->steps);
  ui_draw_line(oled, 2, buffer, UI_COLOR_WHITE);
  format_distance(value, sizeof(value), trip->distance_m);
  snprintf(buffer, sizeof(buffer), "Dist: %s", value);
  ui_draw_line(oled, 3, buffer, UI_COLOR_WHITE);

  if (!trip->sensor_ok) {
    ui_draw_line(oled, 4, "No IMU", UI_COLOR_RED);
    clear_rows(oled, 5, UI_CONTENT_ROWS - 1);
    return;
  }
  if (trip->walking) {
    snprintf(buffer, sizeof(buffer), "Spd: %.1f km/h", (double)trip->speed_kmh);
    ui_draw_line(oled, 4, buffer, UI_COLOR_WHITE);
    snprintf(buffer, sizeof(buffer), "Cad: %.0f /min",
             (double)trip->cadence_spm);
    ui_draw_line(oled, 6, buffer, UI_COLOR_WHITE);
    snprintf(buffer, sizeof(buffer), "Step: %.2f m", (double)trip->last_step_m);
    ui_draw_line(oled, 7, buffer, UI_COLOR_WHITE);
  } else {
    ui_draw_line(oled, 4, "Spd: ---", UI_COLOR_GRAY);
    ui_draw_line(oled, 6, "Cad: ---", UI_COLOR_GRAY);
    ui_draw_line(oled, 7, "Step: ---", UI_COLOR_GRAY);
  }
  snprintf(buffer, sizeof(buffer), "Avg: %.1f km/h",
           (double)trip->avg_speed_kmh);
  ui_draw_line(oled, 5, buffer, UI_COLOR_WHITE);
  ui_draw_line(oled, 8, trip->walking ? "Now: walking" : "Now: still",
               trip->walking ? UI_COLOR_GREEN : UI_COLOR_GRAY);
  snprintf(buffer, sizeof(buffer), "K: %.2f", (double)trip->weinberg_k);
  ui_draw_line(oled, 9, buffer, UI_COLOR_GRAY);
}

static void format_offset(char *buffer, size_t size, const char *axis,
                          float metres) {
  if (fabsf(metres) < 9999.95f) {
    snprintf(buffer, size, "%s: %+.1f m", axis, (double)metres);
  } else {
    snprintf(buffer, size, "%s: %+.2f km", axis, (double)(metres / 1000.0f));
  }
}

static float angle_delta(float target_deg, float from_deg) {
  float delta = fmodf(target_deg - from_deg, 360.0f);
  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta < -180.0f) {
    delta += 360.0f;
  }
  return delta;
}

void ui_draw_pdr_card(ssd1351_t *oled, const ui_pdr_t *pdr) {
  static const char *const state_names[] = {
      [UI_TRIP_STOPPED] = "PDR: STOP",
      [UI_TRIP_RUNNING] = "PDR: RUN",
      [UI_TRIP_PAUSED] = "PDR: PAUSE",
  };
  static const uint16_t state_colors[] = {
      [UI_TRIP_STOPPED] = UI_COLOR_GRAY,
      [UI_TRIP_RUNNING] = UI_COLOR_GREEN,
      [UI_TRIP_PAUSED] = UI_COLOR_YELLOW,
  };
  char buffer[32];
  char value[16];
  ui_draw_line(oled, 0, state_names[pdr->state], state_colors[pdr->state]);
  if (!pdr->sensor_ok) {
    ui_draw_line(oled, 1, "No IMU", UI_COLOR_RED);
    clear_rows(oled, 2, UI_CONTENT_ROWS - 1);
    return;
  }

  format_offset(buffer, sizeof(buffer), "E", pdr->east_m);
  ui_draw_line(oled, 1, buffer, UI_COLOR_WHITE);
  format_offset(buffer, sizeof(buffer), "N", pdr->north_m);
  ui_draw_line(oled, 2, buffer, UI_COLOR_WHITE);
  format_distance(value, sizeof(value), pdr->range_m);
  snprintf(buffer, sizeof(buffer), "Rng: %s", value);
  ui_draw_line(oled, 3, buffer, UI_COLOR_WHITE);

  if (pdr->bearing_valid) {
    float back = pdr->bearing_deg + 180.0f;
    snprintf(buffer, sizeof(buffer), "Brg: %u %s",
             rounded_degrees(pdr->bearing_deg), cardinal(pdr->bearing_deg));
    ui_draw_line(oled, 4, buffer, UI_COLOR_WHITE);
    snprintf(buffer, sizeof(buffer), "Back: %u %s", rounded_degrees(back),
             cardinal(back));
    ui_draw_line(oled, 5, buffer, UI_COLOR_WHITE);
    if (pdr->heading_valid) {
      float turn = angle_delta(back, pdr->heading_deg);
      long turn_deg = lroundf(fabsf(turn));
      if (turn_deg <= 10) {
        snprintf(buffer, sizeof(buffer), "Turn: ahead");
      } else {
        snprintf(buffer, sizeof(buffer), "Turn: %c %ld",
                 (turn < 0.0f) ? 'L' : 'R', turn_deg);
      }
      ui_draw_line(oled, 6, buffer, UI_COLOR_WHITE);
    } else {
      ui_draw_line(oled, 6, "Turn: ---", UI_COLOR_GRAY);
    }
  } else {
    ui_draw_line(oled, 4, "Brg: ---", UI_COLOR_GRAY);
    ui_draw_line(oled, 5, "Back: ---", UI_COLOR_GRAY);
    ui_draw_line(oled, 6, "Turn: ---", UI_COLOR_GRAY);
  }

  if (pdr->heading_valid) {
    snprintf(buffer, sizeof(buffer), "Hdg: %u %s",
             rounded_degrees(pdr->heading_deg), cardinal(pdr->heading_deg));
    ui_draw_line(oled, 7, buffer, UI_COLOR_WHITE);
  } else {
    ui_draw_line(oled, 7, "Hdg: ---", UI_COLOR_GRAY);
  }
  format_distance(value, sizeof(value), pdr->path_m);
  snprintf(buffer, sizeof(buffer), "Path: %s", value);
  ui_draw_line(oled, 8, buffer, UI_COLOR_WHITE);

  if (pdr->steps_no_heading > 0) {
    snprintf(buffer, sizeof(buffer), "No hdg: %lu st",
             (unsigned long)pdr->steps_no_heading);
    ui_draw_line(oled, 9, buffer, UI_COLOR_YELLOW);
  } else if (!pdr->mag_calibrated) {
    ui_draw_line(oled, 9, "Mag: no cal", UI_COLOR_YELLOW);
  } else {
    static const char *const refs[] = {
        [UI_COMPASS_REF_LEVEL] = "Ref: lvl",
        [UI_COMPASS_REF_TOP] = "Ref: top",
        [UI_COMPASS_REF_BACK] = "Ref: back",
    };
    ui_draw_line(oled, 9, refs[pdr->reference], UI_COLOR_GRAY);
  }
}

void ui_draw_rows(ssd1351_t *oled, const ui_row_t *rows) {
  for (uint8_t row = 0; row < UI_CONTENT_ROWS; row++) {
    ui_draw_line(oled, row, rows[row].text, rows[row].color);
  }
}

void ui_format_distance(char *buffer, size_t size, float metres) {
  format_distance(buffer, size, metres);
}

unsigned ui_round_degrees(float degrees) { return rounded_degrees(degrees); }

const char *ui_cardinal(float degrees) { return cardinal(degrees); }

void ui_draw_backpack_card(ssd1351_t *oled, bool rtc_time_valid,
                           uint16_t interval_seconds) {
  char buffer[32];
  ui_draw_line(oled, 0, "Backpack mode", UI_COLOR_WHITE);
  ui_draw_line(oled, 1, "", UI_COLOR_WHITE);
  snprintf(buffer, sizeof(buffer), "Interval: %u s", interval_seconds);
  ui_draw_line(oled, 2, buffer, UI_COLOR_WHITE);
  ui_draw_line(oled, 3, "Screen sleeps", UI_COLOR_GRAY);
  ui_draw_line(oled, 4, "OK: show time", UI_COLOR_GRAY);
  ui_draw_line(oled, 5, "BACK: exit", UI_COLOR_GRAY);
  ui_draw_line(oled, 6, "", UI_COLOR_WHITE);
  if (rtc_time_valid) {
    ui_draw_line(oled, 7, "", UI_COLOR_WHITE);
    ui_draw_line(oled, 8, "", UI_COLOR_WHITE);
  } else {
    ui_draw_line(oled, 7, "RTC invalid", UI_COLOR_RED);
    ui_draw_line(oled, 8, "Sync on SAT", UI_COLOR_YELLOW);
  }
  ui_draw_line(oled, 9, "", UI_COLOR_WHITE);
}

void ui_draw_backpack_view(ssd1351_t *oled, const ds3231_datetime_t *utc,
                           bool sd_error) {
  char buffer[32];
  ui_set_shift(0, 0);
  ui_clear(oled);
  ui_draw_line(oled, 0, "Backpack mode", UI_COLOR_WHITE);
  if (utc != NULL) {
    ds3231_datetime_t local;
    datetime_to_local(utc, &local, CENTURY_BASE);
    format_date(buffer, sizeof(buffer), local.day, local.month, local.year);
    ui_draw_centered(oled, 2, buffer, UI_COLOR_WHITE);
    snprintf(buffer, sizeof(buffer), "%02u:%02u", local.hours, local.minutes);
    ui_draw_centered(oled, 4, buffer, UI_COLOR_WHITE);
  } else {
    ui_draw_centered(oled, 2, "--.--.----", UI_COLOR_RED);
    ui_draw_centered(oled, 4, "--:--", UI_COLOR_RED);
  }
  ui_draw_line(oled, 6, "BACK: exit", UI_COLOR_GRAY);
  if (sd_error) {
    ui_draw_line(oled, 7, "SD error", UI_COLOR_RED);
  }
}
