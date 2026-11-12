#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <stdbool.h>
#include <stdint.h>

#define WAYPOINTS_SLOT_SIZE 256u
#define WAYPOINTS_RECORD_SIZE 48u

enum {
  WAYPOINT_HAS_LATLON = 1u << 0,
  WAYPOINT_HAS_ALTITUDE = 1u << 1,
  WAYPOINT_HAS_TIME = 1u << 2,
};

typedef struct {
  uint16_t id;
  uint16_t flags;
  uint32_t session;
  uint32_t utc_s;
  int32_t lat_e7;
  int32_t lon_e7;
  float altitude_m;
  float east_m;
  float north_m;
  float track_m;
} waypoint_t;

typedef struct {
  void (*read)(void *ctx, uint16_t slot, uint8_t *data, uint16_t length);
  bool (*program)(void *ctx, uint16_t slot, const uint8_t *data);
  bool (*erase_all)(void *ctx);
  void *ctx;
  uint16_t slot_count;
} wp_storage_t;

typedef struct {
  wp_storage_t storage;
  waypoint_t *items;
  uint16_t count;
  uint16_t next_slot;
  uint16_t damaged;
} waypoints_t;

uint16_t waypoints_load(waypoints_t *w, const wp_storage_t *storage,
                        waypoint_t *items);

bool waypoints_add(waypoints_t *w, waypoint_t *wp);

bool waypoints_clear(waypoints_t *w);

bool waypoints_full(const waypoints_t *w);

uint32_t waypoints_crc32(const uint8_t *data, uint16_t length);
void waypoints_encode(const waypoint_t *wp, uint8_t *record);
bool waypoints_decode(const uint8_t *record, waypoint_t *wp);

#endif // WAYPOINTS_H
