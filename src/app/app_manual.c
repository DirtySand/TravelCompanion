#include "app.h"

#define MANUAL_REFRESH_MS 1000
#define REFRESH_PHASE_SHIFT_MS 500

static void handle_back(app_t *app) {
  if (app->mag_calibrating) {
    cancel_mag_calibration(app);
    return;
  }
  if (app->card != CARD_ENVIRONMENT) {
    app->card = CARD_ENVIRONMENT;
    app->clear_screen = true;
  }
}

static void handle_ok(app_t *app) {
  switch (app->card) {
  case CARD_SATELLITES:
    toggle_auto_sync(app);
    break;
  case CARD_COMPASS:
    if (app->mag_calibrating) {
      finish_mag_calibration(app);
    } else {
      start_mag_calibration(app);
    }
    break;
  case CARD_TRIP:
  case CARD_PDR:
  case CARD_ALT:
    trip_ok(app);
    break;
  case CARD_WPT:
  case CARD_DIST:
  case CARD_LINE:
  case CARD_AREA:
    tools_ok(app);
    break;
  case CARD_BACKPACK:
    buttons_wait_release(BUTTON_OK);
    enter_backpack_mode(app);
    break;
  default:
    break;
  }
}

static void handle_manual_buttons(app_t *app, uint32_t pressed) {
  if (tools_handle_select(app, pressed)) {
    pressed &= ~(BUTTON_MASK(BUTTON_NEXT_CARD) | BUTTON_MASK(BUTTON_PREV_CARD) |
                 BUTTON_MASK(BUTTON_BACK) | BUTTON_MASK(BUTTON_OK));
  }
  if (pressed & BUTTON_LONG_MASK(BUTTON_BACK)) {
    gps_set_enabled(app, !app->gps_enabled);
  }
  if (pressed & BUTTON_MASK(BUTTON_BACK)) {
    handle_back(app);
  }
  if (pressed & BUTTON_MASK(BUTTON_OK)) {
    handle_ok(app);
  }
  if (pressed & BUTTON_LONG_MASK(BUTTON_OK)) {
    if (app->card == CARD_TRIP || app->card == CARD_PDR ||
        app->card == CARD_ALT) {
      trip_long_ok(app);
    } else {
      tools_long_ok(app);
    }
  }
  if (app->mode != MODE_MANUAL) {
    return;
  }
  if (pressed & BUTTON_MASK(BUTTON_NEXT_CARD)) {
    app->card = (card_t)((app->card + 1) % CARD_COUNT);
    app->clear_screen = true;
    app->confirm = CONFIRM_NONE;
  }
  if (pressed & BUTTON_MASK(BUTTON_PREV_CARD)) {
    app->card = (card_t)((app->card + CARD_COUNT - 1) % CARD_COUNT);
    app->clear_screen = true;
    app->confirm = CONFIRM_NONE;
  }
  if (app->card != CARD_COMPASS) {
    cancel_mag_calibration(app);
  }
}

void run_manual_cycle(app_t *app, absolute_time_t *next_refresh) {
  gps_service(app);
  gps_commands_service(app);
  check_time_sync_timeout(app);
  if (rtc_alarm_pending() && !handle_sample_alarm(app)) {
    show_error(app, "RTC error");
  }
  handle_manual_buttons(app, buttons_take_events());
  if (app->mode != MODE_MANUAL) {
    return;
  }
  update_shift(app);
  nav_update(app);

  bool scheduled = time_reached(*next_refresh);
  bool fast_due =
      fast_refresh_ms(app) != 0 && time_reached(app->fast_refresh_at);
  if (app->clear_screen || app->refresh_now || scheduled || fast_due) {
    app->refresh_now = false;
    draw_current_card(app, scheduled);
  }
  if (scheduled) {
    *next_refresh = delayed_by_ms(*next_refresh, MANUAL_REFRESH_MS);
    if (app->refresh_resync) {
      *next_refresh = delayed_by_ms(*next_refresh, REFRESH_PHASE_SHIFT_MS);
      app->refresh_resync = false;
    }
    if (time_reached(*next_refresh)) {
      *next_refresh = make_timeout_time_ms(MANUAL_REFRESH_MS);
      app->shown_second_valid = false;
    }
  }

  absolute_time_t wake_at = *next_refresh;
  if (fast_refresh_ms(app) != 0 &&
      absolute_time_diff_us(app->fast_refresh_at, wake_at) > 0) {
    wake_at = app->fast_refresh_at;
  }
  while (!buttons_has_events() && !app->refresh_now && !rtc_alarm_pending() &&
         !best_effort_wfe_or_timeout(wake_at)) {
    gps_service(app);
    gps_commands_service(app);
  }
}
