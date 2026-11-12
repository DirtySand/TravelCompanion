#include "waypoints.h"
#include <string.h>

#define RECORD_MAGIC 0x50574354u
#define ERASED_WORD 0xFFFFFFFFu

uint32_t waypoints_crc32(const uint8_t *data, uint16_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return ~crc;
}

static void put32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static uint32_t get32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void put_float(uint8_t *p, float v) {
  uint32_t bits;
  memcpy(&bits, &v, sizeof(bits));
  put32(p, bits);
}

static float get_float(const uint8_t *p) {
  uint32_t bits = get32(p);
  float v;
  memcpy(&v, &bits, sizeof(v));
  return v;
}

void waypoints_encode(const waypoint_t *wp, uint8_t *record) {
  memset(record, 0, WAYPOINTS_RECORD_SIZE);
  put32(record + 0, RECORD_MAGIC);
  put32(record + 4, (uint32_t)wp->id | ((uint32_t)wp->flags << 16));
  put32(record + 8, wp->session);
  put32(record + 12, wp->utc_s);
  put32(record + 16, (uint32_t)wp->lat_e7);
  put32(record + 20, (uint32_t)wp->lon_e7);
  put_float(record + 24, wp->altitude_m);
  put_float(record + 28, wp->east_m);
  put_float(record + 32, wp->north_m);
  put_float(record + 36, wp->track_m);
  put32(record + 44, waypoints_crc32(record, 44));
}

bool waypoints_decode(const uint8_t *record, waypoint_t *wp) {
  if (get32(record) != RECORD_MAGIC ||
      get32(record + 44) != waypoints_crc32(record, 44)) {
    return false;
  }
  uint32_t word = get32(record + 4);
  wp->id = (uint16_t)word;
  wp->flags = (uint16_t)(word >> 16);
  wp->session = get32(record + 8);
  wp->utc_s = get32(record + 12);
  wp->lat_e7 = (int32_t)get32(record + 16);
  wp->lon_e7 = (int32_t)get32(record + 20);
  wp->altitude_m = get_float(record + 24);
  wp->east_m = get_float(record + 28);
  wp->north_m = get_float(record + 32);
  wp->track_m = get_float(record + 36);
  return true;
}

static bool slot_erased(const uint8_t *record) {
  for (uint16_t i = 0; i < WAYPOINTS_RECORD_SIZE; i++) {
    if (record[i] != 0xFF) {
      return false;
    }
  }
  return true;
}

uint16_t waypoints_load(waypoints_t *w, const wp_storage_t *storage,
                        waypoint_t *items) {
  w->storage = *storage;
  w->items = items;
  w->count = 0;
  w->damaged = 0;
  w->next_slot = storage->slot_count;
  uint8_t record[WAYPOINTS_RECORD_SIZE];
  for (uint16_t slot = 0; slot < storage->slot_count; slot++) {
    storage->read(storage->ctx, slot, record, sizeof(record));
    if (slot_erased(record)) {
      w->next_slot = slot;
      break;
    }
    if (waypoints_decode(record, &items[w->count])) {
      w->count++;
    } else {
      w->damaged++;
    }
  }
  return w->count;
}

bool waypoints_full(const waypoints_t *w) {
  return w->next_slot >= w->storage.slot_count;
}

bool waypoints_add(waypoints_t *w, waypoint_t *wp) {
  if (waypoints_full(w)) {
    return false;
  }
  uint16_t id = 0;
  for (uint16_t i = 0; i < w->count; i++) {
    if (w->items[i].id >= id) {
      id = (uint16_t)(w->items[i].id + 1u);
    }
  }
  wp->id = id;
  uint8_t slot_data[WAYPOINTS_SLOT_SIZE];
  memset(slot_data, 0xFF, sizeof(slot_data));
  waypoints_encode(wp, slot_data);
  uint16_t slot = w->next_slot;
  w->next_slot++;
  if (!w->storage.program(w->storage.ctx, slot, slot_data)) {
    return false;
  }
  uint8_t check[WAYPOINTS_RECORD_SIZE];
  w->storage.read(w->storage.ctx, slot, check, sizeof(check));
  if (memcmp(check, slot_data, sizeof(check)) != 0) {
    return false;
  }
  w->items[w->count++] = *wp;
  return true;
}

bool waypoints_clear(waypoints_t *w) {
  bool ok = w->storage.erase_all(w->storage.ctx);
  w->count = 0;
  w->damaged = 0;
  w->next_slot = 0;
  return ok;
}
