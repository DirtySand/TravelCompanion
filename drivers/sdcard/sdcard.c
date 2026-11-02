#include "sdcard.h"
#include <string.h>

// SD Physical Layer Simplified Specification, SPI mode.

#define SD_CMD0_GO_IDLE_STATE 0
#define SD_CMD8_SEND_IF_COND 8
#define SD_CMD16_SET_BLOCKLEN 16
#define SD_CMD17_READ_SINGLE_BLOCK 17
#define SD_CMD24_WRITE_BLOCK 24
#define SD_CMD55_APP_CMD 55
#define SD_CMD58_READ_OCR 58
#define SD_ACMD41_SD_SEND_OP_COND 41

#define SD_CMD_START_BITS 0x40u
#define SD_CMD8_ARG 0x000001AAu
#define SD_CMD8_ECHO 0xAAu
#define SD_CMD8_VOLTAGE_MASK 0x0Fu
#define SD_ACMD41_HCS 0x40000000u
#define SD_OCR_CCS 0x40u

#define SD_R1_IDLE 0x01u
#define SD_R1_ILLEGAL_COMMAND 0x04u
#define SD_R1_INVALID 0x80u
#define SD_R1_MAX_POLLS 10u

#define SD_TOKEN_START_BLOCK 0xFEu
#define SD_DATA_RESPONSE_MASK 0x1Fu
#define SD_DATA_ACCEPTED 0x05u
#define SD_IDLE_BYTE 0xFFu

#define SD_POWER_UP_BYTES 10u
#define SD_CMD0_RETRIES 10u
#define SD_INIT_TIMEOUT_MS 1000u
#define SD_READ_TOKEN_TIMEOUT_MS 200u
#define SD_BUSY_TIMEOUT_MS 500u

struct sdcard {
  sdcard_io_t io;
  bool ready;
  bool block_addressing;
};

static sdcard_t instance;
static bool instance_used = false;

sdcard_t *sdcard_create(const sdcard_io_t *io) {
  if (io == NULL || io->transfer == NULL || io->select == NULL ||
      io->set_clock == NULL || io->millis == NULL || instance_used) {
    return NULL;
  }
  memset(&instance, 0, sizeof(instance));
  instance.io = *io;
  instance_used = true;
  return &instance;
}

static uint8_t xfer_byte(sdcard_t *dev, uint8_t value) {
  uint8_t rx;
  dev->io.transfer(dev->io.ctx, &value, &rx, 1);
  return rx;
}

static void select_card(sdcard_t *dev) { dev->io.select(dev->io.ctx, true); }

static void deselect_card(sdcard_t *dev) {
  dev->io.select(dev->io.ctx, false);
  // One extra byte with CS high lets the card release DO.
  xfer_byte(dev, SD_IDLE_BYTE);
}

static bool elapsed(sdcard_t *dev, uint32_t start, uint32_t timeout_ms) {
  return (uint32_t)(dev->io.millis(dev->io.ctx) - start) >= timeout_ms;
}

static bool wait_not_busy(sdcard_t *dev, uint32_t timeout_ms) {
  uint32_t start = dev->io.millis(dev->io.ctx);
  while (xfer_byte(dev, SD_IDLE_BYTE) != SD_IDLE_BYTE) {
    if (elapsed(dev, start, timeout_ms)) {
      return false;
    }
  }
  return true;
}

static uint8_t crc7(const uint8_t *data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    uint8_t byte = data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (uint8_t)(crc << 1);
      if ((byte ^ crc) & 0x80u) {
        crc ^= 0x09u;
      }
      byte = (uint8_t)(byte << 1);
    }
  }
  return (uint8_t)((crc << 1) | 0x01u);
}

// Leaves the card selected; caller reads any trailing bytes and deselects.
static uint8_t send_command(sdcard_t *dev, uint8_t command, uint32_t argument) {
  select_card(dev);
  if (command != SD_CMD0_GO_IDLE_STATE &&
      !wait_not_busy(dev, SD_BUSY_TIMEOUT_MS)) {
    return SD_IDLE_BYTE;
  }
  uint8_t frame[6];
  frame[0] = (uint8_t)(SD_CMD_START_BITS | command);
  frame[1] = (uint8_t)(argument >> 24);
  frame[2] = (uint8_t)(argument >> 16);
  frame[3] = (uint8_t)(argument >> 8);
  frame[4] = (uint8_t)argument;
  frame[5] = crc7(frame, 5);
  dev->io.transfer(dev->io.ctx, frame, NULL, sizeof(frame));

  uint8_t r1 = SD_IDLE_BYTE;
  for (uint8_t i = 0; i < SD_R1_MAX_POLLS; i++) {
    r1 = xfer_byte(dev, SD_IDLE_BYTE);
    if ((r1 & SD_R1_INVALID) == 0) {
      break;
    }
  }
  return r1;
}

static uint8_t send_app_command(sdcard_t *dev, uint8_t command,
                                uint32_t argument) {
  uint8_t r1 = send_command(dev, SD_CMD55_APP_CMD, 0);
  deselect_card(dev);
  if (r1 & ~SD_R1_IDLE) {
    return r1;
  }
  return send_command(dev, command, argument);
}

static sdcard_status_t fail(sdcard_t *dev, sdcard_status_t status) {
  deselect_card(dev);
  dev->ready = false;
  // The SPI bus may be shared (OLED): never leave it at the slow init clock.
  dev->io.set_clock(dev->io.ctx, true);
  return status;
}

sdcard_status_t sdcard_init(sdcard_t *dev) {
  if (dev == NULL) {
    return SDCARD_NULL_PTR;
  }
  dev->ready = false;
  dev->block_addressing = false;
  dev->io.set_clock(dev->io.ctx, false);

  // At least 74 clocks with CS high (spec 6.4.1.1).
  dev->io.select(dev->io.ctx, false);
  for (uint8_t i = 0; i < SD_POWER_UP_BYTES; i++) {
    xfer_byte(dev, SD_IDLE_BYTE);
  }

  uint8_t r1 = SD_IDLE_BYTE;
  for (uint8_t i = 0; i < SD_CMD0_RETRIES && r1 != SD_R1_IDLE; i++) {
    r1 = send_command(dev, SD_CMD0_GO_IDLE_STATE, 0);
    deselect_card(dev);
  }
  if (r1 != SD_R1_IDLE) {
    return fail(dev, SDCARD_NO_CARD);
  }

  bool version2 = false;
  r1 = send_command(dev, SD_CMD8_SEND_IF_COND, SD_CMD8_ARG);
  if (r1 == SD_R1_IDLE) {
    uint8_t r7[4];
    dev->io.transfer(dev->io.ctx, NULL, r7, sizeof(r7));
    deselect_card(dev);
    if ((r7[2] & SD_CMD8_VOLTAGE_MASK) != 0x01u || r7[3] != SD_CMD8_ECHO) {
      return fail(dev, SDCARD_UNSUPPORTED);
    }
    version2 = true;
  } else if (r1 & SD_R1_ILLEGAL_COMMAND) {
    deselect_card(dev);
  } else {
    return fail(dev, SDCARD_IO_ERROR);
  }

  uint32_t start = dev->io.millis(dev->io.ctx);
  do {
    r1 = send_app_command(dev, SD_ACMD41_SD_SEND_OP_COND,
                          version2 ? SD_ACMD41_HCS : 0);
    deselect_card(dev);
    if (r1 == 0) {
      break;
    }
    if (r1 == SD_IDLE_BYTE) {
      return fail(dev, SDCARD_IO_ERROR);
    }
    if (r1 & ~SD_R1_IDLE) {
      // MMC cards reject ACMD41; they are not supported.
      return fail(dev, SDCARD_UNSUPPORTED);
    }
  } while (!elapsed(dev, start, SD_INIT_TIMEOUT_MS));
  if (r1 != 0) {
    return fail(dev, SDCARD_TIMEOUT);
  }

  if (version2) {
    r1 = send_command(dev, SD_CMD58_READ_OCR, 0);
    uint8_t ocr[4];
    dev->io.transfer(dev->io.ctx, NULL, ocr, sizeof(ocr));
    deselect_card(dev);
    if (r1 != 0) {
      return fail(dev, SDCARD_IO_ERROR);
    }
    dev->block_addressing = (ocr[0] & SD_OCR_CCS) != 0;
  }
  if (!dev->block_addressing) {
    r1 = send_command(dev, SD_CMD16_SET_BLOCKLEN, SDCARD_BLOCK_SIZE);
    deselect_card(dev);
    if (r1 != 0) {
      return fail(dev, SDCARD_IO_ERROR);
    }
  }

  dev->io.set_clock(dev->io.ctx, true);
  dev->ready = true;
  return SDCARD_OK;
}

bool sdcard_is_ready(const sdcard_t *dev) { return dev != NULL && dev->ready; }

void sdcard_invalidate(sdcard_t *dev) {
  if (dev != NULL) {
    dev->ready = false;
  }
}

static uint32_t block_address(const sdcard_t *dev, uint32_t lba) {
  return dev->block_addressing ? lba : lba * SDCARD_BLOCK_SIZE;
}

sdcard_status_t sdcard_read_block(sdcard_t *dev, uint32_t lba, uint8_t *data) {
  if (dev == NULL || data == NULL) {
    return SDCARD_NULL_PTR;
  }
  if (!dev->ready) {
    return SDCARD_NOT_READY;
  }
  if (send_command(dev, SD_CMD17_READ_SINGLE_BLOCK, block_address(dev, lba)) !=
      0) {
    return fail(dev, SDCARD_IO_ERROR);
  }
  uint32_t start = dev->io.millis(dev->io.ctx);
  uint8_t token;
  while ((token = xfer_byte(dev, SD_IDLE_BYTE)) == SD_IDLE_BYTE) {
    if (elapsed(dev, start, SD_READ_TOKEN_TIMEOUT_MS)) {
      return fail(dev, SDCARD_TIMEOUT);
    }
  }
  if (token != SD_TOKEN_START_BLOCK) {
    return fail(dev, SDCARD_IO_ERROR);
  }
  dev->io.transfer(dev->io.ctx, NULL, data, SDCARD_BLOCK_SIZE);
  uint8_t crc[2];
  dev->io.transfer(dev->io.ctx, NULL, crc, sizeof(crc));
  deselect_card(dev);
  return SDCARD_OK;
}

sdcard_status_t sdcard_write_block(sdcard_t *dev, uint32_t lba,
                                   const uint8_t *data) {
  if (dev == NULL || data == NULL) {
    return SDCARD_NULL_PTR;
  }
  if (!dev->ready) {
    return SDCARD_NOT_READY;
  }
  if (send_command(dev, SD_CMD24_WRITE_BLOCK, block_address(dev, lba)) != 0) {
    return fail(dev, SDCARD_IO_ERROR);
  }
  const uint8_t header[2] = {SD_IDLE_BYTE, SD_TOKEN_START_BLOCK};
  dev->io.transfer(dev->io.ctx, header, NULL, sizeof(header));
  dev->io.transfer(dev->io.ctx, data, NULL, SDCARD_BLOCK_SIZE);
  const uint8_t crc[2] = {SD_IDLE_BYTE, SD_IDLE_BYTE};
  dev->io.transfer(dev->io.ctx, crc, NULL, sizeof(crc));

  uint8_t response = xfer_byte(dev, SD_IDLE_BYTE);
  if ((response & SD_DATA_RESPONSE_MASK) != SD_DATA_ACCEPTED) {
    return fail(dev, SDCARD_IO_ERROR);
  }
  // Wait for the internal programming to finish before releasing the card,
  // so it is never busy when the MCU goes dormant.
  if (!wait_not_busy(dev, SD_BUSY_TIMEOUT_MS)) {
    return fail(dev, SDCARD_TIMEOUT);
  }
  deselect_card(dev);
  return SDCARD_OK;
}

sdcard_status_t sdcard_wait_ready(sdcard_t *dev) {
  if (dev == NULL) {
    return SDCARD_NULL_PTR;
  }
  if (!dev->ready) {
    return SDCARD_NOT_READY;
  }
  select_card(dev);
  bool idle = wait_not_busy(dev, SD_BUSY_TIMEOUT_MS);
  deselect_card(dev);
  return idle ? SDCARD_OK : SDCARD_TIMEOUT;
}