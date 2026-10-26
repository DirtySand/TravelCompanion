#include "ui.h"
#include "board_config.h"
#include "datetime_utils.h"
#include <stdio.h>
#include <string.h>

void ui_clear(ssd1351_t *oled) { ssd1351_fill(oled, UI_COLOR_BLACK); }

void ui_draw_line(ssd1351_t *oled, uint8_t row, const char *text,
                  uint16_t color) {
  char line[UI_COLUMNS + 1];
  size_t length = strlen(text);
  if (length > UI_COLUMNS) {
    length = UI_COLUMNS;
  }
  memcpy(line, text, length);
  memset(line + length, ' ', UI_COLUMNS - length);
  line[UI_COLUMNS] = '\0';
  ssd1351_draw_string(oled, 0, (uint8_t)(row * UI_ROW_HEIGHT), line, color,
                      UI_COLOR_BLACK);
}

void ui_draw_status(ssd1351_t *oled, const char *text, uint16_t color) {
  ui_draw_line(oled, UI_STATUS_ROW, text, color);
}

void ui_draw_footer(ssd1351_t *oled, const char *title, uint8_t index,
                    uint8_t count) {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u/%u %s", index, count, title);
  ui_draw_line(oled, UI_FOOTER_ROW, buffer, UI_COLOR_GRAY);
}

void ui_draw_environment_card(ssd1351_t *oled, const ui_environment_t *env,
                              const ds3231_datetime_t *utc,
                              bool rtc_time_valid) {
  char buffer[32];
  if (env->valid) {
    snprintf(buffer, sizeof(buffer),
             "Temp: %.1f"
             "\x7f"
             "C",
             env->temperature);
    ui_draw_line(oled, 0, buffer, UI_COLOR_WHITE);
    snprintf(buffer, sizeof(buffer), "Pres: %.1f hPa", env->pressure / 100);
    ui_draw_line(oled, 1, buffer, UI_COLOR_WHITE);
    snprintf(buffer, sizeof(buffer), "Hum:  %.2f %%", env->humidity);
    ui_draw_line(oled, 2, buffer, UI_COLOR_WHITE);
  } else {
    ui_draw_line(oled, 0, "Sensor error", UI_COLOR_RED);
    ui_draw_line(oled, 1, "", UI_COLOR_WHITE);
    ui_draw_line(oled, 2, "", UI_COLOR_WHITE);
  }

  if (utc != NULL) {
    ds3231_datetime_t local;
    datetime_to_local(utc, &local, CENTURY_BASE);
    snprintf(buffer, sizeof(buffer), "Time: %02u:%02u:%02u", local.hours,
             local.minutes, local.seconds);
    ui_draw_line(oled, 3, buffer, UI_COLOR_WHITE);
    snprintf(buffer, sizeof(buffer), "Date: %02u-%02u-%04u", local.day,
             local.month, CENTURY_BASE + local.year);
    ui_draw_line(oled, 4, buffer, UI_COLOR_WHITE);
  } else {
    ui_draw_line(oled, 3, "Time: --:--:--", UI_COLOR_RED);
    ui_draw_line(oled, 4, "Date: --", UI_COLOR_RED);
  }
  ui_draw_line(oled, 5, rtc_time_valid ? "" : "RTC time invalid", UI_COLOR_RED);
}

static void format_coordinate(char *buffer, size_t size, const char *label,
                              int32_t value_e7, char positive, char negative) {
  uint32_t magnitude =
      (value_e7 < 0) ? (uint32_t)(-(int64_t)value_e7) : (uint32_t)value_e7;
  snprintf(buffer, size, "%s %c %lu.%05lu", label,
           (value_e7 < 0) ? negative : positive,
           (unsigned long)(magnitude / 10000000u),
           (unsigned long)((magnitude % 10000000u) / 100u));
}

static const char *gps_state_text(ui_gps_state_t state, uint16_t *color) {
  switch (state) {
  case UI_GPS_FIX:
    *color = UI_COLOR_GREEN;
    return "GPS: FIX";
  case UI_GPS_LOST:
    *color = UI_COLOR_YELLOW;
    return "GPS: SIGNAL LOST";
  case UI_GPS_SEARCHING:
    *color = UI_COLOR_YELLOW;
    return "GPS: SEARCHING";
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

static void draw_gps_position(ssd1351_t *oled, const ui_gps_t *gps,
                              uint16_t color) {
  char buffer[32];
  const neo6m_data_t *f = &gps->fix;

  if (gps->has_fix && f->position_valid) {
    format_coordinate(buffer, sizeof(buffer), "Lat:", f->latitude_e7, 'N', 'S');
    ui_draw_line(oled, 2, buffer, color);
    format_coordinate(buffer, sizeof(buffer), "Lon:", f->longitude_e7, 'E',
                      'W');
    ui_draw_line(oled, 3, buffer, color);
  } else {
    ui_draw_line(oled, 2, "Lat: ---", color);
    ui_draw_line(oled, 3, "Lon: ---", color);
  }

  if (gps->has_fix && f->altitude_valid) {
    int32_t alt = f->altitude_dm;
    uint32_t magnitude = (alt < 0) ? (uint32_t)(-(int64_t)alt) : (uint32_t)alt;
    snprintf(buffer, sizeof(buffer), "Alt: %s%lu.%lu m", (alt < 0) ? "-" : "",
             (unsigned long)(magnitude / 10u),
             (unsigned long)(magnitude % 10u));
    ui_draw_line(oled, 4, buffer, color);
  } else {
    ui_draw_line(oled, 4, "Alt: ---", color);
  }

  if (gps->has_fix && f->speed_valid) {
    snprintf(buffer, sizeof(buffer), "Spd: %lu.%lu km/h",
             (unsigned long)(f->speed_kmh_x10 / 10u),
             (unsigned long)(f->speed_kmh_x10 % 10u));
    ui_draw_line(oled, 5, buffer, color);
  } else {
    ui_draw_line(oled, 5, "Spd: ---", color);
  }

  if (gps->has_fix && f->course_valid) {
    snprintf(buffer, sizeof(buffer),
             "Crs: %u.%u"
             "\x7f",
             f->course_x100 / 100u, (f->course_x100 % 100u) / 10u);
    ui_draw_line(oled, 6, buffer, color);
  } else {
    ui_draw_line(oled, 6, "Crs: ---", color);
  }

  if (gps->has_fix && f->hdop_valid) {
    snprintf(buffer, sizeof(buffer), "HDOP: %u.%02u", f->hdop_x100 / 100u,
             f->hdop_x100 % 100u);
    ui_draw_line(oled, 7, buffer, color);
  } else {
    ui_draw_line(oled, 7, "HDOP: ---", color);
  }
}

void ui_draw_gps_card(ssd1351_t *oled, const ui_gps_t *gps) {
  char buffer[32];
  const neo6m_data_t *live = &gps->live;
  uint16_t color;

  const char *state = gps_state_text(gps->state, &color);
  ui_draw_line(oled, 0, state, color);

  uint16_t live_color =
      gps_is_receiving(gps->state) ? UI_COLOR_WHITE : UI_COLOR_GRAY;
  uint16_t fix_color =
      (gps->state == UI_GPS_FIX) ? UI_COLOR_WHITE : UI_COLOR_GRAY;

  if (!live->sky_valid) {
    snprintf(buffer, sizeof(buffer), "Sats: %u", live->satellites);
  } else if (live->cno_top4_avg == 0) {
    snprintf(buffer, sizeof(buffer), "Sat %u/%u CN0 --", live->satellites,
             live->satellites_in_view);
  } else {
    snprintf(buffer, sizeof(buffer), "Sat %u/%u CN0 %u", live->satellites,
             live->satellites_in_view, live->cno_top4_avg);
  }
  ui_draw_line(oled, 1, buffer, live_color);

  draw_gps_position(oled, gps, fix_color);

  if (live->time_valid) {
    snprintf(buffer, sizeof(buffer), "UTC:  %02u:%02u:%02u", live->hours,
             live->minutes, live->seconds);
    ui_draw_line(oled, 8, buffer, live_color);
  } else {
    ui_draw_line(oled, 8, "UTC:  --:--:--", live_color);
  }

  if (live->date_valid) {
    snprintf(buffer, sizeof(buffer), "Date: %02u-%02u-%04u", live->day,
             live->month, CENTURY_BASE + live->year);
    ui_draw_line(oled, 9, buffer, live_color);
  } else {
    ui_draw_line(oled, 9, "Date: --", live_color);
  }

  if (gps->has_fix && gps->state != UI_GPS_FIX) {
    char age[16];
    if (gps->fix_age_valid) {
      format_age(age, sizeof(age), gps->fix_age_s);
    } else {
      snprintf(age, sizeof(age), "?");
    }
    snprintf(buffer, sizeof(buffer), "Fix age: %s", age);
    ui_draw_line(oled, 10, buffer, UI_COLOR_GRAY);
  } else {
    unsigned long errors =
        (unsigned long)(gps->stats.checksum_errors + gps->stats.overflows +
                        gps->dropped_bytes);
    snprintf(buffer, sizeof(buffer), "Rx %lu Err %lu",
             (unsigned long)gps->stats.sentences, errors);
    ui_draw_line(oled, 10, buffer, UI_COLOR_GRAY);
  }
}

void ui_draw_backpack_card(ssd1351_t *oled, bool rtc_time_valid,
                           uint16_t interval_seconds) {
  char buffer[32];
  ui_draw_line(oled, 0, "Backpack mode", UI_COLOR_WHITE);
  snprintf(buffer, sizeof(buffer), "Interval: %u s", interval_seconds);
  ui_draw_line(oled, 2, buffer, UI_COLOR_WHITE);
  ui_draw_line(oled, 3, "Screen dimmed", UI_COLOR_GRAY);
  ui_draw_line(oled, 4, "BACK: exit", UI_COLOR_GRAY);
  if (rtc_time_valid) {
    ui_draw_line(oled, 6, "", UI_COLOR_WHITE);
    ui_draw_line(oled, 7, "", UI_COLOR_WHITE);
  } else {
    ui_draw_line(oled, 6, "RTC time invalid", UI_COLOR_RED);
    ui_draw_line(oled, 7, "Sync on GPS card", UI_COLOR_YELLOW);
  }
}