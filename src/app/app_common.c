#include "app.h"
#include "board_config.h"
#include "datetime_utils.h"
#include "timezone.h"
#include <stdio.h>

#define STATUS_MESSAGE_MS 4000

// ---------------------------------------------------------------------------
// Status line
// ---------------------------------------------------------------------------

void set_status(app_t *app, const char *text, uint16_t color) {
  snprintf(app->status.text, sizeof(app->status.text), "%s", text);
  app->status.color = color;
  app->status.expires = make_timeout_time_ms(STATUS_MESSAGE_MS);
  app->refresh_now = true;
}

void show_error(app_t *app, const char *message) {
  printf("%s\n", message);
  set_status(app, message, UI_COLOR_RED);
}

bool status_active(const app_t *app) {
  return app->status.text[0] != '\0' && !time_reached(app->status.expires);
}

int64_t ms_since(absolute_time_t since) {
  return absolute_time_diff_us(since, get_absolute_time()) / 1000;
}

// ---------------------------------------------------------------------------
// Time helpers (RTC seconds since 2000-01-01 UTC)
// ---------------------------------------------------------------------------

bool rtc_now(app_t *app, ds3231_datetime_t *utc, uint32_t *seconds) {
  if (app->time_invalid ||
      ds3231_read_datetime(app->ds3231, utc) != DS3231_OK) {
    return false;
  }
  *seconds = datetime_to_seconds(utc, CENTURY_BASE);
  return true;
}

void seconds_to_datetime(uint32_t seconds, ds3231_datetime_t *out) {
  *out = (ds3231_datetime_t){.day = 1, .month = 1, .year = 0};
  while (seconds > 86400u * 365u) {
    datetime_add_seconds(out, (int)(86400u * 365u), CENTURY_BASE);
    seconds -= 86400u * 365u;
  }
  datetime_add_seconds(out, (int)seconds, CENTURY_BASE);
}

int32_t local_offset_s(uint32_t utc_s) {
  ds3231_datetime_t utc;
  seconds_to_datetime(utc_s, &utc);
  int32_t offset = TIMEZONE_STANDARD_OFFSET_SECONDS;
  if (is_dst_active(utc.day, utc.month, (uint16_t)(CENTURY_BASE + utc.year),
                    utc.hours)) {
    offset += TIMEZONE_DST_OFFSET_SECONDS;
  }
  return offset;
}

uint16_t local_minutes(int64_t utc_s) {
  int64_t local = utc_s + local_offset_s((uint32_t)utc_s);
  int64_t of_day = local % 86400;
  if (of_day < 0) {
    of_day += 86400;
  }
  return (uint16_t)(of_day / 60);
}
