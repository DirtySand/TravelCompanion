#include "ui.h"
#include "board_config.h"
#include "datetime_utils.h"
#include <stdio.h>
#include <string.h>

// Custom degree glyph in font8x8. Kept as a separate literal so the \x escape
// cannot swallow following hex-valid characters.
#define DEG "\x7f"

static uint8_t shift_x = 0;
static uint8_t shift_y = 0;

void ui_set_shift(uint8_t dx, uint8_t dy) {
  shift_x = (dx > UI_SHIFT_MAX_X) ? UI_SHIFT_MAX_X : dx;
  shift_y = (dy > UI_SHIFT_MAX_Y) ? UI_SHIFT_MAX_Y : dy;
}

void ui_clear(ssd1351_t *oled) { ssd1351_fill(oled, UI_COLOR_BLACK); }

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
  // Clear only the gaps so the text itself never flickers.
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
