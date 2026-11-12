#include "sd_log.h"
#include "board_config.h"
#include "sdcard.h"
#include "sdcard_port.h"
#include "ff.h"
#include "log_crypto.h"
#include "log_pubkey.h"
#include "monocypher.h"
#include "pico/rand.h"
#include "sample.h"
#include "sd_diskio.h"
#include <stdio.h>
#include <string.h>

#define SD_LOG_BUFFER_SIZE 4096u
#define SD_LOG_RETRY_SECONDS 60u
#define SD_LOG_NAME_PATTERN "TC*.TCL"
#define SD_LOG_NAME_FORMAT "TC%06lu.TCL"
#define SD_LOG_NAME_DIGITS 6u
#define SD_LOG_NAME_PREFIX_LENGTH 2u
#define SD_LOG_MAX_INDEX 999999ul

static sdcard_t *card;
static FATFS fs;
static FIL file;
static log_crypto_t crypto;
static uint8_t buffer[SD_LOG_BUFFER_SIZE];
static size_t buffered;
static sd_log_state_t state = SD_LOG_NO_CARD;
static bool file_open;
static bool attempted;
static uint32_t last_attempt_s;
static uint32_t last_flush_s;

void sd_log_init(void) {
  sdcard_port_hw_init();
  sdcard_io_t io = sdcard_port_get_io();
  card = sdcard_create(&io);
  sd_diskio_attach(card);
}

sd_log_state_t sd_log_state(void) { return state; }

static void shut_down(sd_log_state_t new_state) {
  if (file_open) {
    f_close(&file);
    file_open = false;
  }
  f_mount(NULL, "", 0);
  log_crypto_end(&crypto);
  sdcard_invalidate(card);
  buffered = 0;
  state = new_state;
}

static bool parse_index(const char *name, unsigned long *index) {
  unsigned long value = 0;
  for (uint8_t i = 0; i < SD_LOG_NAME_DIGITS; i++) {
    char c = name[SD_LOG_NAME_PREFIX_LENGTH + i];
    if (c < '0' || c > '9') {
      return false;
    }
    value = value * 10u + (unsigned long)(c - '0');
  }
  *index = value;
  return true;
}

static bool next_file_index(unsigned long *next) {
  DIR dir;
  FILINFO info;
  unsigned long highest = 0;
  FRESULT result = f_findfirst(&dir, &info, "", SD_LOG_NAME_PATTERN);
  while (result == FR_OK && info.fname[0] != '\0') {
    unsigned long index;
    if (parse_index(info.fname, &index) && index > highest) {
      highest = index;
    }
    result = f_findnext(&dir, &info);
  }
  f_closedir(&dir);
  if (result != FR_OK || highest >= SD_LOG_MAX_INDEX) {
    return false;
  }
  *next = highest + 1u;
  return true;
}

static void random_seed(uint8_t seed[LOG_CRYPTO_SEED_SIZE]) {
  for (size_t offset = 0; offset < LOG_CRYPTO_SEED_SIZE;
       offset += sizeof(rng_128_t)) {
    rng_128_t random;
    get_rand_128(&random);
    memcpy(&seed[offset], &random, sizeof(random));
    crypto_wipe(&random, sizeof(random));
  }
}

static bool write_all(const void *data, size_t length) {
  UINT written = 0;
  return f_write(&file, data, (UINT)length, &written) == FR_OK &&
         written == length;
}

static bool flush_buffer(uint32_t now_s) {
  if (buffered > 0 && !write_all(buffer, buffered)) {
    return false;
  }
  buffered = 0;
  if (f_sync(&file) != FR_OK) {
    return false;
  }
  last_flush_s = now_s;
  return true;
}

static bool seal_line(const char *line, size_t length) {
  size_t written = log_crypto_seal(&crypto, (const uint8_t *)line, length,
                                   &buffer[buffered],
                                   sizeof(buffer) - buffered);
  if (written == 0) {
    return false;
  }
  buffered += written;
  return true;
}

static void open_log(uint32_t now_s) {
  attempted = true;
  last_attempt_s = now_s;

  FRESULT result = f_mount(&fs, "", 1);
  if (result != FR_OK) {
    printf("SD mount failed: %d\n", result);
    shut_down((result == FR_NOT_READY) ? SD_LOG_NO_CARD : SD_LOG_ERROR);
    return;
  }

  unsigned long index;
  char name[16];
  if (!next_file_index(&index)) {
    shut_down(SD_LOG_ERROR);
    return;
  }
  snprintf(name, sizeof(name), SD_LOG_NAME_FORMAT, index);
  if (f_open(&file, name, FA_WRITE | FA_CREATE_NEW) != FR_OK) {
    shut_down(SD_LOG_ERROR);
    return;
  }
  file_open = true;

  uint8_t seed[LOG_CRYPTO_SEED_SIZE];
  random_seed(seed);
  log_crypto_begin(&crypto, LOG_RECIPIENT_PUBLIC_KEY, seed);
  crypto_wipe(seed, sizeof(seed));

  char header[SAMPLE_CSV_MAX];
  size_t header_length = sample_csv_header(header, sizeof(header));
  buffered = 0;
  if (!write_all(crypto.header, LOG_CRYPTO_HEADER_SIZE) ||
      !seal_line(header, header_length) || !flush_buffer(now_s)) {
    shut_down(SD_LOG_ERROR);
    return;
  }
  printf("SD log: %s\n", name);
  state = SD_LOG_READY;
}

void sd_log_append(const char *line, size_t length,
                   const ds3231_datetime_t *utc, bool time_valid,
                   uint32_t now_s) {
  sd_diskio_set_time(utc, time_valid);
  if (state != SD_LOG_READY) {
    if (attempted && (uint32_t)(now_s - last_attempt_s) < SD_LOG_RETRY_SECONDS) {
      return;
    }
    open_log(now_s);
    if (state != SD_LOG_READY) {
      return;
    }
  }
  if (sizeof(buffer) - buffered < LOG_CRYPTO_RECORD_OVERHEAD + length &&
      !flush_buffer(now_s)) {
    shut_down(SD_LOG_ERROR);
    return;
  }
  if (!seal_line(line, length)) {
    return;
  }
  if ((uint32_t)(now_s - last_flush_s) >= LOG_FLUSH_INTERVAL_SECONDS &&
      !flush_buffer(now_s)) {
    printf("SD write failed\n");
    shut_down(SD_LOG_ERROR);
  }
}

bool sd_log_flush(uint32_t now_s) {
  if (state != SD_LOG_READY) {
    return false;
  }
  if (!flush_buffer(now_s)) {
    shut_down(SD_LOG_ERROR);
    return false;
  }
  return true;
}
