#include "app.h"
#include "board_config.h"
#include "neo6m_port.h"
#include "power.h"
#include "sd_log.h"

#define BACKPACK_VIEW_MS 2000

static void show_backpack_view(app_t *app) {
  ds3231_datetime_t now;
  bool ok = ds3231_read_datetime(app->ds3231, &now) == DS3231_OK;
  ssd1351_set_contrast(app->oled, OLED_CONTRAST_BACKPACK);
  ui_draw_backpack_view(app->oled, ok ? &now : NULL,
                        sd_log_state() == SD_LOG_ERROR);
  ssd1351_set_sleep(app->oled, false);
  app->view_active = true;
  app->view_deadline = make_timeout_time_ms(BACKPACK_VIEW_MS);
}

static void hide_backpack_view(app_t *app) {
  ssd1351_set_sleep(app->oled, true);
  app->view_active = false;
}

void enter_backpack_mode(app_t *app) {
  if (app->time_invalid) {
    set_status(app, "Sync time first", UI_COLOR_RED);
    return;
  }
  if (!app->schedule_valid && !schedule_from_now(app)) {
    show_error(app, "RTC alarm error");
    return;
  }
  gps_commands_finish(app);
  mag_set_running(app, false);
  imu_set_running(app, false);
  app->mode = MODE_BACKPACK;
  app->sync_pending = false;
  show_backpack_view(app);
}

static void leave_backpack_mode(app_t *app) {
  app->mode = MODE_MANUAL;
  mag_set_running(app, true);
  imu_set_running(app, true);
  app->view_active = false;
  ssd1351_set_contrast(app->oled, OLED_CONTRAST_MANUAL);
  ssd1351_set_sleep(app->oled, false);
  apply_shift(app);
  app->shift_at = make_timeout_time_ms(BURN_IN_SHIFT_MS);
  app->clear_screen = true;
  app->shown_second_valid = false;
  buttons_take_events();
}

static void abort_backpack_mode(app_t *app, const char *message) {
  leave_backpack_mode(app);
  show_error(app, message);
}

static uint32_t wait_while_view_active(app_t *app) {
  while (!buttons_has_events() && !rtc_alarm_pending() &&
         !best_effort_wfe_or_timeout(app->view_deadline)) {
    gps_service(app);
  }
  if (time_reached(app->view_deadline)) {
    hide_backpack_view(app);
  }
  return buttons_take_events();
}

static uint32_t sleep_until_event(app_t *app) {
  uint8_t ok_pin = buttons_pin(BUTTON_OK);
  uint8_t back_pin = buttons_pin(BUTTON_BACK);
  uint32_t wake = power_dormant_until(
      RTC_INT_PIN, POWER_PIN_MASK(ok_pin) | POWER_PIN_MASK(back_pin));
  neo6m_port_hw_init();

  uint32_t events = 0;
  if (wake & POWER_PIN_MASK(ok_pin)) {
    buttons_wait_release(BUTTON_OK);
    events |= BUTTON_MASK(BUTTON_OK);
  }
  if (wake & POWER_PIN_MASK(back_pin)) {
    buttons_wait_release(BUTTON_BACK);
    events |= BUTTON_MASK(BUTTON_BACK);
  }
  return events;
}

void run_backpack_cycle(app_t *app) {
  uint32_t events =
      app->view_active ? wait_while_view_active(app) : sleep_until_event(app);

  if (rtc_alarm_pending() && !handle_sample_alarm(app)) {
    abort_backpack_mode(app, "RTC error");
    return;
  }
  if (events & (BUTTON_MASK(BUTTON_BACK) | BUTTON_LONG_MASK(BUTTON_BACK))) {
    leave_backpack_mode(app);
    return;
  }
  if (events & (BUTTON_MASK(BUTTON_OK) | BUTTON_LONG_MASK(BUTTON_OK))) {
    show_backpack_view(app);
  }
}
