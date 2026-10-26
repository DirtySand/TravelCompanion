#ifndef UI_H
#define UI_H

#include "drivers/SSD1351U3/ssd1351.h"
#include "drivers/ds3231/ds3231.h"
#include "drivers/neo6m/neo6m.h"
#include <stdbool.h>
#include <stdint.h>

#define UI_COLUMNS 16
#define UI_ROW_HEIGHT 10
#define UI_STATUS_ROW 11
#define UI_FOOTER_ROW 12

#define UI_COLOR_BLACK 0x0000
#define UI_COLOR_WHITE 0xFFFF
#define UI_COLOR_RED 0xF800
#define UI_COLOR_GREEN 0x07E0
#define UI_COLOR_YELLOW 0xFFE0
#define UI_COLOR_GRAY 0x8410

typedef struct {
  bool valid;
  float temperature;
  float pressure;
  float humidity;
} ui_environment_t;

typedef enum {
  UI_GPS_NO_DATA,
  UI_GPS_TIMEOUT,
  UI_GPS_SEARCHING,
  UI_GPS_LOST,
  UI_GPS_FIX,
} ui_gps_state_t;

typedef struct {
  ui_gps_state_t state;
  bool has_fix;
  bool fix_age_valid;
  uint32_t fix_age_s;
  neo6m_data_t live;
  neo6m_data_t fix;
  neo6m_stats_t stats;
  uint32_t dropped_bytes;
} ui_gps_t;

void ui_clear(ssd1351_t *oled);
void ui_draw_line(ssd1351_t *oled, uint8_t row, const char *text,
                  uint16_t color);
void ui_draw_status(ssd1351_t *oled, const char *text, uint16_t color);
void ui_draw_footer(ssd1351_t *oled, const char *title, uint8_t index,
                    uint8_t count);
void ui_draw_environment_card(ssd1351_t *oled, const ui_environment_t *env,
                              const ds3231_datetime_t *utc,
                              bool rtc_time_valid);
void ui_draw_gps_card(ssd1351_t *oled, const ui_gps_t *gps);
void ui_draw_backpack_card(ssd1351_t *oled, bool rtc_time_valid,
                           uint16_t interval_seconds);

#endif // UI_H