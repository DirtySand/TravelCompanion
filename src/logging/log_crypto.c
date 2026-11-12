#include "log_crypto.h"
#include "monocypher.h"
#include <string.h>

#define MAGIC "TCLOGv01"
#define MAGIC_SIZE 8u
#define EPHEMERAL_OFFSET MAGIC_SIZE
#define NONCE_PREFIX_OFFSET (EPHEMERAL_OFFSET + LOG_CRYPTO_KEY_SIZE)
#define NONCE_PREFIX_SIZE 16u
#define NONCE_SIZE 24u
#define SEED_HASH_SIZE 64u

static void put_u64_le(uint8_t *out, uint64_t value) {
  for (uint8_t i = 0; i < 8; i++) {
    out[i] = (uint8_t)(value >> (8u * i));
  }
}

void log_crypto_begin(log_crypto_t *ctx,
                      const uint8_t recipient_public_key[LOG_CRYPTO_KEY_SIZE],
                      const uint8_t seed[LOG_CRYPTO_SEED_SIZE]) {
  memset(ctx, 0, sizeof(*ctx));

  // Whiten the raw entropy: 32 B ephemeral secret + 16 B nonce prefix.
  uint8_t material[SEED_HASH_SIZE];
  crypto_blake2b(material, sizeof(material), seed, LOG_CRYPTO_SEED_SIZE);
  uint8_t *ephemeral_secret = material;

  uint8_t *header = ctx->header;
  memcpy(header, MAGIC, MAGIC_SIZE);
  crypto_x25519_public_key(&header[EPHEMERAL_OFFSET], ephemeral_secret);
  memcpy(&header[NONCE_PREFIX_OFFSET], &material[LOG_CRYPTO_KEY_SIZE],
         NONCE_PREFIX_SIZE);

  uint8_t kdf_input[3 * LOG_CRYPTO_KEY_SIZE];
  crypto_x25519(kdf_input, ephemeral_secret, recipient_public_key);
  memcpy(&kdf_input[LOG_CRYPTO_KEY_SIZE], &header[EPHEMERAL_OFFSET],
         LOG_CRYPTO_KEY_SIZE);
  memcpy(&kdf_input[2 * LOG_CRYPTO_KEY_SIZE], recipient_public_key,
         LOG_CRYPTO_KEY_SIZE);
  crypto_blake2b(ctx->key, sizeof(ctx->key), kdf_input, sizeof(kdf_input));

  crypto_wipe(material, sizeof(material));
  crypto_wipe(kdf_input, sizeof(kdf_input));
  ctx->record_index = 0;
  ctx->active = true;
}

size_t log_crypto_seal(log_crypto_t *ctx, const uint8_t *plain_text,
                       size_t length, uint8_t *out, size_t out_size) {
  if (ctx == NULL || !ctx->active || plain_text == NULL || out == NULL ||
      length == 0 || length > LOG_CRYPTO_MAX_PLAINTEXT ||
      out_size < LOG_CRYPTO_RECORD_OVERHEAD + length) {
    return 0;
  }
  uint8_t nonce[NONCE_SIZE];
  memcpy(nonce, &ctx->header[NONCE_PREFIX_OFFSET], NONCE_PREFIX_SIZE);
  put_u64_le(&nonce[NONCE_PREFIX_SIZE], ctx->record_index);

  out[0] = (uint8_t)length;
  out[1] = (uint8_t)(length >> 8);
  crypto_aead_lock(&out[LOG_CRYPTO_RECORD_OVERHEAD], &out[LOG_CRYPTO_LENGTH_SIZE],
                   ctx->key, nonce, ctx->header, LOG_CRYPTO_HEADER_SIZE,
                   plain_text, length);
  ctx->record_index++;
  return LOG_CRYPTO_RECORD_OVERHEAD + length;
}

void log_crypto_end(log_crypto_t *ctx) {
  if (ctx != NULL) {
    crypto_wipe(ctx, sizeof(*ctx));
  }
}
