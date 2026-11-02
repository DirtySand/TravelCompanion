#ifndef SDCARD_H
#define SDCARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SDCARD_BLOCK_SIZE 512u

typedef struct sdcard sdcard_t;

typedef enum {
  SDCARD_OK,
  SDCARD_NULL_PTR,
  SDCARD_NO_CARD,
  SDCARD_UNSUPPORTED,
  SDCARD_TIMEOUT,
  SDCARD_IO_ERROR,
  SDCARD_NOT_READY,
} sdcard_status_t;

// tx == NULL sends 0xFF, rx == NULL discards received bytes.
typedef void (*sdcard_transfer_fn)(void *ctx, const uint8_t *tx, uint8_t *rx,
                                   size_t length);
typedef void (*sdcard_select_fn)(void *ctx, bool selected);
typedef void (*sdcard_set_clock_fn)(void *ctx, bool fast);
typedef uint32_t (*sdcard_millis_fn)(void *ctx);

typedef struct {
  sdcard_transfer_fn transfer;
  sdcard_select_fn select;
  sdcard_set_clock_fn set_clock;
  sdcard_millis_fn millis;
  void *ctx;
} sdcard_io_t;

sdcard_t *sdcard_create(const sdcard_io_t *io);
sdcard_status_t sdcard_init(sdcard_t *dev);
bool sdcard_is_ready(const sdcard_t *dev);
void sdcard_invalidate(sdcard_t *dev);
sdcard_status_t sdcard_read_block(sdcard_t *dev, uint32_t lba, uint8_t *data);
sdcard_status_t sdcard_write_block(sdcard_t *dev, uint32_t lba,
                                   const uint8_t *data);
sdcard_status_t sdcard_wait_ready(sdcard_t *dev);

#endif // SDCARD_H
