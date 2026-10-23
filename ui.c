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

static const char *gps_fix_text(const ui_gps_t *gps, uint16_t *color) {
  if (!gps->received) {
    *color = UI_COLOR_RED;
    return "GPS: NO DATA";
  }
  if (!gps->fresh) {
    *color = UI_COLOR_RED;
    return "GPS: TIMEOUT";
  }
  if (!gps->data.fix_valid) {
    *color = UI_COLOR_YELLOW;
    return "GPS: NO FIX";
  }
  *color = UI_COLOR_GREEN;
  return "GPS: FIX";
}

void ui_draw_gps_card(ssd1351_t *oled, const ui_gps_t *gps) {
  char buffer[32];
  const neo6m_data_t *d = &gps->data;
  uint16_t color;

  const char *fix = gps_fix_text(gps, &color);
  ui_draw_line(oled, 0, fix, color);

  color = gps->fresh ? UI_COLOR_WHITE : UI_COLOR_GRAY;

  snprintf(buffer, sizeof(buffer), "Sats: %u", d->satellites);
  ui_draw_line(oled, 1, buffer, color);

  if (d->position_valid) {
    format_coordinate(buffer, sizeof(buffer), "Lat:", d->latitude_e7, 'N', 'S');
    ui_draw_line(oled, 2, buffer, color);
    format_coordinate(buffer, sizeof(buffer), "Lon:", d->longitude_e7, 'E',
                      'W');
    ui_draw_line(oled, 3, buffer, color);
  } else {
    ui_draw_line(oled, 2, "Lat: ---", color);
    ui_draw_line(oled, 3, "Lon: ---", color);
  }

  if (d->altitude_valid) {
    int32_t alt = d->altitude_dm;
    uint32_t magnitude = (alt < 0) ? (uint32_t)(-(int64_t)alt) : (uint32_t)alt;
    snprintf(buffer, sizeof(buffer), "Alt: %s%lu.%lu m", (alt < 0) ? "-" : "",
             (unsigned long)(magnitude / 10u),
             (unsigned long)(magnitude % 10u));
    ui_draw_line(oled, 4, buffer, color);
  } else {
    ui_draw_line(oled, 4, "Alt: ---", color);
  }

  if (d->speed_valid) {
    snprintf(buffer, sizeof(buffer), "Spd: %lu.%lu km/h",
             (unsigned long)(d->speed_kmh_x10 / 10u),
             (unsigned long)(d->speed_kmh_x10 % 10u));
    ui_draw_line(oled, 5, buffer, color);
  } else {
    ui_draw_line(oled, 5, "Spd: ---", color);
  }

  if (d->course_valid) {
    snprintf(buffer, sizeof(buffer),
             "Crs: %u.%u"
             "\x7f",
             d->course_x100 / 100u, (d->course_x100 % 100u) / 10u);
    ui_draw_line(oled, 6, buffer, color);
  } else {
    ui_draw_line(oled, 6, "Crs: ---", color);
  }

  if (d->hdop_valid) {
    snprintf(buffer, sizeof(buffer), "HDOP: %u.%02u", d->hdop_x100 / 100u,
             d->hdop_x100 % 100u);
    ui_draw_line(oled, 7, buffer, color);
  } else {
    ui_draw_line(oled, 7, "HDOP: ---", color);
  }

  if (d->time_valid) {
    snprintf(buffer, sizeof(buffer), "UTC:  %02u:%02u:%02u", d->hours,
             d->minutes, d->seconds);
    ui_draw_line(oled, 8, buffer, color);
  } else {
    ui_draw_line(oled, 8, "UTC:  --:--:--", color);
  }

  if (d->date_valid) {
    snprintf(buffer, sizeof(buffer), "Date: %02u-%02u-%04u", d->day, d->month,
             CENTURY_BASE + d->year);
    ui_draw_line(oled, 9, buffer, color);
  } else {
    ui_draw_line(oled, 9, "Date: --", color);
  }

  unsigned long errors =
      (unsigned long)(gps->stats.checksum_errors + gps->stats.overflows +
                      gps->dropped_bytes);
  snprintf(buffer, sizeof(buffer), "Rx %lu Err %lu",
           (unsigned long)gps->stats.sentences, errors);
  ui_draw_line(oled, 10, buffer, UI_COLOR_GRAY);
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