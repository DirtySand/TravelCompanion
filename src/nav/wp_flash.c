#include "wp_flash.h"
#include "hardware/flash.h"
#include "pico/flash.h"
#include <string.h>

#define REGION_OFFSET (PICO_FLASH_SIZE_BYTES - WP_FLASH_REGION_SIZE)
#define SAFE_TIMEOUT_MS 100

extern char __flash_binary_end;

typedef struct {
  uint32_t offset;
  const uint8_t *data;
} flash_op_t;

static void read_slot(void *ctx, uint16_t slot, uint8_t *data,
                      uint16_t length) {
  (void)ctx;
  const uint8_t *src = (const uint8_t *)(XIP_BASE + REGION_OFFSET +
                                         (uint32_t)slot * WAYPOINTS_SLOT_SIZE);
  memcpy(data, src, length);
}

static void do_program(void *param) {
  const flash_op_t *op = (const flash_op_t *)param;
  flash_range_program(op->offset, op->data, FLASH_PAGE_SIZE);
}

static void do_erase(void *param) {
  const flash_op_t *op = (const flash_op_t *)param;
  flash_range_erase(op->offset, FLASH_SECTOR_SIZE);
}

static bool program_slot(void *ctx, uint16_t slot, const uint8_t *data) {
  (void)ctx;
  flash_op_t op = {
      .offset = REGION_OFFSET + (uint32_t)slot * WAYPOINTS_SLOT_SIZE,
      .data = data,
  };
  return flash_safe_execute(do_program, &op, SAFE_TIMEOUT_MS) == PICO_OK;
}

static bool erase_all(void *ctx) {
  (void)ctx;
  for (uint32_t offset = 0; offset < WP_FLASH_REGION_SIZE;
       offset += FLASH_SECTOR_SIZE) {
    flash_op_t op = {.offset = REGION_OFFSET + offset, .data = NULL};
    if (flash_safe_execute(do_erase, &op, SAFE_TIMEOUT_MS) != PICO_OK) {
      return false;
    }
  }
  return true;
}

bool wp_flash_storage(wp_storage_t *storage) {
  _Static_assert(WAYPOINTS_SLOT_SIZE == FLASH_PAGE_SIZE,
                 "one waypoint record per flash page");
  _Static_assert(WP_FLASH_REGION_SIZE % FLASH_SECTOR_SIZE == 0,
                 "region must be whole sectors");
  uintptr_t binary_end = (uintptr_t)&__flash_binary_end;
  if (binary_end > XIP_BASE + REGION_OFFSET) {
    return false;
  }
  *storage = (wp_storage_t){
      .read = read_slot,
      .program = program_slot,
      .erase_all = erase_all,
      .ctx = NULL,
      .slot_count = (uint16_t)WP_FLASH_SLOTS,
  };
  return true;
}
