#ifndef APP_H
#define APP_H

#include "alt.h"
#include "attitude.h"
#include "bme680.h"
#include "buttons.h"
#include "cards.h"
#include "compass.h"
#include "ds3231.h"
#include "fusion.h"
#include "icm20649.h"
#include "measure.h"
#include "neo6m.h"
#include "pdr.h"
#include "pico/stdlib.h"
#include "qmc5883l.h"
#include "sht40.h"
#include "ssd1351.h"
#include "steps.h"
#include "trip.h"
#include "ui.h"
#include "waypoints.h"
#include <stdbool.h>
#include <stdint.h>

#define BURN_IN_SHIFT_MS 60000
#define COMPASS_MIN_HORIZONTAL_UT 2.0f
#define MS_TO_KMH 3.6f
#define PDR_MIN_RANGE_M 1.0f
#define AREA_MAX_POINTS 32

#define BUTTON_PREV_CARD BUTTON_BOTTOM_LEFT
#define BUTTON_NEXT_CARD BUTTON_BOTTOM_RIGHT
#define BUTTON_OK BUTTON_TOP_RIGHT
#define BUTTON_BACK BUTTON_TOP_LEFT

typedef enum {
  MODE_MANUAL,
  MODE_BACKPACK,
} app_mode_t;

typedef enum {
  CARD_ENVIRONMENT,
  CARD_POSITION,
  CARD_SATELLITES,
  CARD_COMPASS,
  CARD_NAV,
  CARD_ALT,
  CARD_TRIP,
  CARD_PDR,
  CARD_WPT,
  CARD_DIST,
  CARD_LINE,
  CARD_AREA,
  CARD_SUN,
  CARD_IMU,
  CARD_BACKPACK,
  CARD_COUNT,
} card_t;

typedef enum {
  GPS_CMD_CONFIGURE = 1u << 0,
  GPS_CMD_POLL_TIME = 1u << 1,
  GPS_CMD_POWER_OFF = 1u << 2,
} gps_command_t;

typedef struct {
  char text[UI_COLUMNS + 1];
  uint16_t color;
  absolute_time_t expires;
} status_message_t;

typedef enum {
  POINT_NONE,
  POINT_HERE,
  POINT_MARK,
  POINT_WAYPOINT,
} point_kind_t;

typedef struct {
  point_kind_t kind;
  uint16_t wp_id;
  measure_point_t pos;
  bool alt_valid;
  float alt_m;
  float track_m;
} tool_point_t;

typedef struct {
  measure_point_t pos;
  bool alt_valid;
  float alt_m;
  bool track_valid;
  float track_m;
} resolved_point_t;

typedef enum {
  SELECT_NONE,
  SELECT_WPT,
  SELECT_DIST_A,
  SELECT_DIST_B,
} select_mode_t;

typedef enum {
  CONFIRM_NONE,
  CONFIRM_WPT_CLEAR,
  CONFIRM_AREA_CLEAR,
} confirm_action_t;

typedef struct {
  ssd1351_t *oled;
  bme680_t *bme680;
  sht40_t *sht40;
  ds3231_t *ds3231;
  neo6m_t *gps;
  qmc5883l_t *mag;
  icm20649_t *imu;
  app_mode_t mode;
  card_t card;
  bool time_invalid;
  bool clear_screen;
  bool refresh_now;
  bool gps_enabled;
  absolute_time_t gps_toggled_at;
  bool gps_received;
  absolute_time_t gps_last_rmc;
  bool gps_has_fix;
  neo6m_data_t gps_fix;
  bool gps_fix_utc_valid;
  uint32_t gps_fix_utc_s;
  uint32_t gps_commands;
  bool gps_command_woken;
  absolute_time_t gps_command_at;
  bool sync_pending;
  absolute_time_t sync_deadline;
  bool auto_sync;
  bool auto_sync_done;
  absolute_time_t time_info_poll_at;
  ds3231_datetime_t next_sample;
  bool schedule_valid;
  uint32_t shift_step;
  absolute_time_t shift_at;
  bool view_active;
  absolute_time_t view_deadline;
  bool shown_second_valid;
  uint8_t shown_second;
  bool refresh_resync;
  status_message_t status;
  bool mag_ok;
  bool mag_error;
  bool mag_received;
  bool mag_overflow;
  bool mag_filter_valid;
  absolute_time_t mag_sampled_at;
  compass_vec_t mag_field;
  compass_cal_t mag_cal;
  bool mag_calibrating;
  compass_cal_session_t mag_session;
  bool imu_ok;
  bool imu_error;
  bool imu_received;
  bool imu_filter_valid;
  absolute_time_t imu_sampled_at;
  compass_vec_t imu_accel_g;
  compass_vec_t imu_gyro_dps;
  float imu_temp_c;
  absolute_time_t fast_refresh_at;
  attitude_t attitude;
  bool imu_fifo_running;
  bool imu_fifo_checked;
  bool mag_fed;
  absolute_time_t mag_fed_at;
  steps_t steps;
  trip_t trip;
  pdr_t pdr;
  fusion_t fusion;
  float dr_east_m;
  float dr_north_m;
  bool dr_steps_no_heading;
  float track_m;
  uint32_t session;
  bool steps_since_fix;
  bool nav_fix_seen;
  absolute_time_t nav_fix_at;
  bool nav_idle_seen;
  absolute_time_t nav_idle_at;
  alt_t alt;
  absolute_time_t baro_at;
  absolute_time_t baro_last;
  bool baro_seen;
  waypoints_t wps;
  bool wps_ok;
  uint16_t wp_index;
  select_mode_t select;
  uint16_t select_index;
  confirm_action_t confirm;
  absolute_time_t confirm_deadline;
  tool_point_t dist_a;
  tool_point_t dist_b;
  uint8_t dist_stage;
  cards_line_stage_t line_stage;
  measure_point_t line_a;
  measure_line_t line;
  float line_base_m;
  measure_point_t area[AREA_MAX_POINTS];
  uint16_t area_count;
} app_t;

// app_common.c
void set_status(app_t *app, const char *text, uint16_t color);
void show_error(app_t *app, const char *message);
bool status_active(const app_t *app);
int64_t ms_since(absolute_time_t since);
bool rtc_now(app_t *app, ds3231_datetime_t *utc, uint32_t *seconds);
void seconds_to_datetime(uint32_t seconds, ds3231_datetime_t *out);
int32_t local_offset_s(uint32_t utc_s);
uint16_t local_minutes(int64_t utc_s);

// app_sampling.c
bool schedule_from_now(app_t *app);
bool read_climate(sht40_t *sht40, float *temperature, float *humidity);
bool read_pressure(bme680_t *bme680, float *pressure);
bool handle_sample_alarm(app_t *app);
bool rtc_alarm_pending(void);

// app_gps.c
bool gps_is_fresh(const app_t *app);
void gps_request(app_t *app, gps_command_t command);
void gps_commands_service(app_t *app);
void gps_commands_finish(app_t *app);
void toggle_auto_sync(app_t *app);
void check_time_sync_timeout(app_t *app);
const char *auto_sync_hint(const app_t *app, uint16_t *color);
void gps_set_enabled(app_t *app, bool enabled);
uint32_t gps_service(app_t *app);
bool gps_wait_for_epoch(app_t *app);
void collect_gps(app_t *app, ui_gps_t *gps);
uint16_t gps_tag_color(const app_t *app);

// app_compass.c
bool mag_init(app_t *app);
void mag_set_running(app_t *app, bool running);
void mag_update(app_t *app);
void start_mag_calibration(app_t *app);
void cancel_mag_calibration(app_t *app);
void finish_mag_calibration(app_t *app);
void draw_compass_card(app_t *app);

// app_imu.c
bool imu_init(app_t *app);
void imu_set_running(app_t *app, bool running);
bool walking_heading(app_t *app, float *heading_deg,
                     ui_compass_ref_t *reference);
void imu_drain_fifo(app_t *app);
void draw_imu_card(app_t *app);

// app_trip.c
ui_trip_state_t ui_trip_state(const trip_t *t);
void draw_trip_card(app_t *app);
void draw_pdr_card(app_t *app);
void trip_ok(app_t *app);
void trip_long_ok(app_t *app);

// app_nav.c
bool nav_heading_ok(const app_t *app);
void nav_steps(app_t *app, float distance_m, bool heading_valid,
               float heading_deg);
void nav_gps(app_t *app);
measure_point_t nav_here(const app_t *app);
measure_point_t nav_dr_here(const app_t *app);
cards_source_t nav_source(const app_t *app);
void nav_update(app_t *app);

// app_waypoints.c
void waypoints_start(app_t *app);
const waypoint_t *waypoint_by_id(const app_t *app, uint16_t id);
bool waypoint_local(const app_t *app, const waypoint_t *wp, measure_point_t *p);
void draw_wpt_card(app_t *app);
void wpt_ok(app_t *app);

// app_tools.c
void confirm_request(app_t *app, confirm_action_t action, const char *text);
bool confirm_pending(app_t *app, confirm_action_t action);
void draw_dist_card(app_t *app);
void draw_line_card(app_t *app);
void draw_area_card(app_t *app);
bool tools_handle_select(app_t *app, uint32_t pressed);
void tools_long_ok(app_t *app);
void tools_ok(app_t *app);
const char *tools_hint(const app_t *app);

// app_cards.c
uint32_t fast_refresh_ms(const app_t *app);
void draw_current_card(app_t *app, bool scheduled);
void apply_shift(const app_t *app);
void update_shift(app_t *app);

// app_backpack.c
void enter_backpack_mode(app_t *app);
void run_backpack_cycle(app_t *app);

// app_manual.c
void run_manual_cycle(app_t *app, absolute_time_t *next_refresh);

#endif // APP_H
