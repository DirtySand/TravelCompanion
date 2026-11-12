#include "app.h"
#include "geo.h"
#include <math.h>
#include <stdio.h>

#define LINE_MIN_BASE_M 2.0f
#define CONFIRM_MS 4000

// ---------------------------------------------------------------------------
// Select mode (BOTTOM buttons = up / down) and confirmations
// ---------------------------------------------------------------------------

void confirm_request(app_t *app, confirm_action_t action,
                            const char *text) {
  app->confirm = action;
  app->confirm_deadline = make_timeout_time_ms(CONFIRM_MS);
  set_status(app, text, UI_COLOR_YELLOW);
}

bool confirm_pending(app_t *app, confirm_action_t action) {
  if (app->confirm != action) {
    return false;
  }
  if (time_reached(app->confirm_deadline)) {
    app->confirm = CONFIRM_NONE;
    return false;
  }
  return true;
}

static uint16_t select_entries(const app_t *app) {
  switch (app->select) {
  case SELECT_WPT:
    return (uint16_t)(app->wps.count + 1u);
  case SELECT_DIST_A:
  case SELECT_DIST_B:
    return (uint16_t)(app->wps.count + 1u);
  default:
    return 0;
  }
}

static void select_move(app_t *app, int step) {
  uint16_t n = select_entries(app);
  if (n == 0) {
    return;
  }
  app->select_index = (uint16_t)((app->select_index + n + step) % n);
  app->refresh_now = true;
}

// ---------------------------------------------------------------------------
// Distance tool (two points, each "here", a mark or a waypoint)
// ---------------------------------------------------------------------------

static tool_point_t point_mark_here(const app_t *app) {
  return (tool_point_t){
      .kind = POINT_MARK,
      .pos = nav_here(app),
      .alt_valid = app->alt.valid,
      .alt_m = alt_altitude_m(&app->alt),
      .track_m = app->track_m,
  };
}

static bool point_resolve(const app_t *app, const tool_point_t *spec,
                          resolved_point_t *out) {
  *out = (resolved_point_t){0};
  switch (spec->kind) {
  case POINT_HERE:
    out->pos = nav_here(app);
    out->alt_valid = app->alt.valid;
    out->alt_m = alt_altitude_m(&app->alt);
    out->track_valid = true;
    out->track_m = app->track_m;
    return true;
  case POINT_MARK:
    out->pos = spec->pos;
    out->alt_valid = spec->alt_valid;
    out->alt_m = spec->alt_m;
    out->track_valid = true;
    out->track_m = spec->track_m;
    return true;
  case POINT_WAYPOINT: {
    const waypoint_t *wp = waypoint_by_id(app, spec->wp_id);
    if (wp == NULL || !waypoint_local(app, wp, &out->pos)) {
      return false;
    }
    out->alt_valid = (wp->flags & WAYPOINT_HAS_ALTITUDE) != 0;
    out->alt_m = wp->altitude_m;
    out->track_valid = wp->session == app->session;
    out->track_m = wp->track_m;
    return true;
  }
  default:
    return false;
  }
}

static void point_label(const tool_point_t *p, char *buffer, size_t size) {
  switch (p->kind) {
  case POINT_HERE:
    snprintf(buffer, size, "here");
    break;
  case POINT_MARK:
    snprintf(buffer, size, "mark");
    break;
  case POINT_WAYPOINT:
    snprintf(buffer, size, "WP%u", p->wp_id);
    break;
  default:
    snprintf(buffer, size, "---");
    break;
  }
}

static tool_point_t dist_entry(const app_t *app, uint16_t index, bool for_a) {
  if (index == 0 || index > app->wps.count) {
    if (for_a) {
      return point_mark_here(app);
    }
    return (tool_point_t){.kind = POINT_HERE};
  }
  return (tool_point_t){.kind = POINT_WAYPOINT,
                        .wp_id = app->wps.items[index - 1u].id};
}

static void dist_ok(app_t *app) {
  if (app->select == SELECT_DIST_A) {
    app->dist_a = dist_entry(app, app->select_index, true);
    app->select = SELECT_DIST_B;
    app->select_index = 0;
    return;
  }
  if (app->select == SELECT_DIST_B) {
    app->dist_b = dist_entry(app, app->select_index, false);
    app->select = SELECT_NONE;
    app->dist_stage = (app->dist_b.kind == POINT_HERE) ? 1 : 2;
    return;
  }
  if (app->dist_stage == 1) {
    app->dist_b = point_mark_here(app);
    app->dist_stage = 2;
    set_status(app, "B marked", UI_COLOR_GREEN);
    return;
  }
  app->dist_a = point_mark_here(app);
  app->dist_b = (tool_point_t){.kind = POINT_HERE};
  app->dist_stage = 1;
  set_status(app, "A marked", UI_COLOR_GREEN);
}

void draw_dist_card(app_t *app) {
  cards_dist_t c = {.selecting = 0};
  tool_point_t a = app->dist_a;
  tool_point_t b = app->dist_b;
  if (app->select == SELECT_DIST_A) {
    a = dist_entry(app, app->select_index, true);
    if (app->select_index == 0) {
      a.kind = POINT_HERE;
    }
    c.selecting = 1;
  } else if (app->select == SELECT_DIST_B) {
    b = dist_entry(app, app->select_index, false);
    c.selecting = 2;
  }
  point_label(&a, c.a_label, sizeof(c.a_label));
  point_label(&b, c.b_label, sizeof(c.b_label));
  resolved_point_t ra, rb;
  if (a.kind != POINT_NONE && point_resolve(app, &a, &ra) &&
      point_resolve(app, &b, &rb)) {
    c.valid = true;
    float de = rb.pos.east_m - ra.pos.east_m;
    float dn = rb.pos.north_m - ra.pos.north_m;
    c.horizontal_m = sqrtf(de * de + dn * dn);
    c.bearing_deg = geo_bearing_deg(de, dn);
    c.dalt_valid = ra.alt_valid && rb.alt_valid;
    c.dalt_m = rb.alt_m - ra.alt_m;
    c.track_valid = ra.track_valid && rb.track_valid;
    c.track_m = fabsf(rb.track_m - ra.track_m);
  }
  if (c.selecting) {
    c.guide = "OK:pick BACK:x";
  } else if (app->dist_stage == 1) {
    c.guide = "B = here (live)";
  }
  ui_row_t rows[UI_CONTENT_ROWS];
  cards_dist(&c, rows);
  ui_draw_rows(app->oled, rows);
}

// ---------------------------------------------------------------------------
// Ruler line (steps only: GPS jumps would tilt a short A-B baseline)
// ---------------------------------------------------------------------------

static void line_ok(app_t *app) {
  if (!nav_heading_ok(app)) {
    show_error(app, "Needs IMU+mag");
    return;
  }
  switch (app->line_stage) {
  case CARDS_LINE_SET_A:
    app->line_a = nav_dr_here(app);
    app->line_stage = CARDS_LINE_SET_B;
    set_status(app, "A set", UI_COLOR_GREEN);
    break;
  case CARDS_LINE_SET_B:
    if (!measure_line_from(app->line_a, nav_dr_here(app), LINE_MIN_BASE_M,
                           &app->line)) {
      show_error(app, "Walk further");
      break;
    }
    app->line_base_m = measure_distance_m(app->line_a, nav_dr_here(app));
    app->line_stage = CARDS_LINE_ACTIVE;
    set_status(app, "Line set", UI_COLOR_GREEN);
    break;
  default:
    break;
  }
}

void draw_line_card(app_t *app) {
  cards_line_t c = {
      .stage = app->line_stage,
      .position_ok = nav_heading_ok(app),
      .min_base_m = LINE_MIN_BASE_M,
      .bearing_deg = app->line.bearing_deg,
      .base_m = app->line_base_m,
  };
  measure_point_t here = nav_dr_here(app);
  c.from_a_m = measure_distance_m(app->line_a, here);
  if (app->line_stage == CARDS_LINE_ACTIVE) {
    measure_line_project(&app->line, here, &c.along_m, &c.offset_m);
  }
  c.heading_valid = walking_heading(app, &c.heading_deg, NULL);
  ui_row_t rows[UI_CONTENT_ROWS];
  cards_line(&c, rows);
  ui_draw_rows(app->oled, rows);
}

// ---------------------------------------------------------------------------
// Area (own points, RAM only)
// ---------------------------------------------------------------------------

static void area_add(app_t *app) {
  if (app->area_count >= AREA_MAX_POINTS) {
    show_error(app, "Area full");
    return;
  }
  app->area[app->area_count++] = nav_here(app);
  char text[UI_COLUMNS + 1];
  snprintf(text, sizeof(text), "Point %u added", app->area_count);
  set_status(app, text, UI_COLOR_GREEN);
}

void draw_area_card(app_t *app) {
  cards_area_t c = {
      .count = app->area_count,
      .max = AREA_MAX_POINTS,
      .source = nav_source(app),
  };
  c.area_m2 = measure_polygon_area_m2(app->area, app->area_count);
  c.perimeter_m = measure_polygon_perimeter_m(app->area, app->area_count);
  c.crossing = measure_polygon_self_intersects(app->area, app->area_count);
  if (app->area_count >= 2) {
    c.last_valid = true;
    c.last_edge_m = measure_distance_m(app->area[app->area_count - 2u],
                                       app->area[app->area_count - 1u]);
  }
  if (app->area_count >= 1) {
    measure_point_t here = nav_here(app);
    c.here_valid = true;
    c.to_first_m = measure_distance_m(here, app->area[0]);
    if (app->area_count < AREA_MAX_POINTS) {
      measure_point_t with_here[AREA_MAX_POINTS];
      for (uint16_t i = 0; i < app->area_count; i++) {
        with_here[i] = app->area[i];
      }
      with_here[app->area_count] = here;
      c.area_with_here_m2 = measure_polygon_area_m2(
          with_here, (uint16_t)(app->area_count + 1u));
    }
  }
  ui_row_t rows[UI_CONTENT_ROWS];
  cards_area(&c, rows);
  ui_draw_rows(app->oled, rows);
}

// ---------------------------------------------------------------------------
// Tool dispatch (WPT, DIST, LINE and AREA cards)
// ---------------------------------------------------------------------------

bool tools_handle_select(app_t *app, uint32_t pressed) {
  if (app->select == SELECT_NONE) {
    return false;
  }
  bool on_card =
      (app->select == SELECT_WPT && app->card == CARD_WPT) ||
      ((app->select == SELECT_DIST_A || app->select == SELECT_DIST_B) &&
       app->card == CARD_DIST);
  if (!on_card) {
    app->select = SELECT_NONE;
    return false;
  }
  if (pressed & BUTTON_MASK(BUTTON_NEXT_CARD)) {
    select_move(app, 1);
  }
  if (pressed & BUTTON_MASK(BUTTON_PREV_CARD)) {
    select_move(app, -1);
  }
  if (pressed & BUTTON_MASK(BUTTON_BACK)) {
    app->select = SELECT_NONE;
    app->refresh_now = true;
  }
  if (pressed & BUTTON_MASK(BUTTON_OK)) {
    if (app->card == CARD_WPT) {
      wpt_ok(app);
    } else {
      dist_ok(app);
    }
    app->refresh_now = true;
  }
  return true;
}

void tools_long_ok(app_t *app) {
  switch (app->card) {
  case CARD_WPT:
    if (app->wps_ok) {
      app->select = SELECT_WPT;
      app->select_index = app->wp_index;
      app->refresh_now = true;
    }
    break;
  case CARD_DIST:
    app->select = SELECT_DIST_A;
    app->select_index = 0;
    app->refresh_now = true;
    break;
  case CARD_LINE:
    app->line_stage = CARDS_LINE_SET_A;
    set_status(app, "Line reset", UI_COLOR_GRAY);
    break;
  case CARD_AREA:
    if (app->area_count > 0) {
      confirm_request(app, CONFIRM_AREA_CLEAR, "OK: clear area?");
    }
    break;
  default:
    break;
  }
}

void tools_ok(app_t *app) {
  switch (app->card) {
  case CARD_WPT:
    wpt_ok(app);
    break;
  case CARD_DIST:
    dist_ok(app);
    break;
  case CARD_LINE:
    line_ok(app);
    break;
  case CARD_AREA:
    if (confirm_pending(app, CONFIRM_AREA_CLEAR)) {
      app->confirm = CONFIRM_NONE;
      app->area_count = 0;
      set_status(app, "Area cleared", UI_COLOR_YELLOW);
    } else {
      area_add(app);
    }
    break;
  default:
    break;
  }
  app->refresh_now = true;
}

const char *tools_hint(const app_t *app) {
  switch (app->card) {
  case CARD_WPT:
    if (app->select == SELECT_WPT) {
      return "</>:pick OK:go";
    }
    return "OK:mark hld:sel";
  case CARD_DIST:
    if (app->select != SELECT_NONE) {
      return "</>:pick OK:go";
    }
    return (app->dist_stage == 1) ? "OK:B hold:sel" : "OK:A hold:sel";
  case CARD_LINE:
    switch (app->line_stage) {
    case CARDS_LINE_SET_A:
      return "OK: set A";
    case CARDS_LINE_SET_B:
      return "OK:B hold:rst";
    default:
      return "Hold OK: reset";
    }
  case CARD_AREA:
    return "OK:add hold:clr";
  default:
    return "";
  }
}
