#ifndef LOG_CRYPTO_H
#define LOG_CRYPTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LOG_CRYPTO_KEY_SIZE 32u
#define LOG_CRYPTO_HEADER_SIZE 64u
#define LOG_CRYPTO_SEED_SIZE 64u
#define LOG_CRYPTO_MAC_SIZE 16u
#define LOG_CRYPTO_LENGTH_SIZE 2u
#define LOG_CRYPTO_RECORD_OVERHEAD (LOG_CRYPTO_LENGTH_SIZE + LOG_CRYPTO_MAC_SIZE)
#define LOG_CRYPTO_MAX_PLAINTEXT 1024u

typedef struct {
  uint8_t key[LOG_CRYPTO_KEY_SIZE];
  uint8_t header[LOG_CRYPTO_HEADER_SIZE];
  uint64_t record_index;
  bool active;
} log_crypto_t;

void log_crypto_begin(log_crypto_t *ctx,
                      const uint8_t recipient_public_key[LOG_CRYPTO_KEY_SIZE],
                      const uint8_t seed[LOG_CRYPTO_SEED_SIZE]);

size_t log_crypto_seal(log_crypto_t *ctx, const uint8_t *plain_text,
                       size_t length, uint8_t *out, size_t out_size);

void log_crypto_end(log_crypto_t *ctx);

#endif // LOG_CRYPTO_H
