#ifndef LOG_CRYPTO_H
#define LOG_CRYPTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Encrypted log file format (all integers little-endian):
//
//   header (64 B):  magic "TCLOGv01" (8) | ephemeral X25519 public key (32) |
//                   nonce prefix (16) | reserved, zero (8)
//   record (n):     plaintext length u16 (2) | Poly1305 MAC (16) |
//                   XChaCha20 ciphertext (length)
//
//   file key = BLAKE2b-256(X25519(ephemeral secret, recipient public) |
//                          ephemeral public | recipient public)
//   nonce    = nonce prefix (16) | record index u64 (8)
//   AEAD     = Monocypher crypto_aead_lock (XChaCha20-Poly1305, RFC 8439
//              construction), associated data = the 64-byte header.
//
// The device only knows the recipient public key, so neither the card nor
// the device itself can decrypt finished files.

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

// seed must come from a true random source; it is wiped by the caller.
void log_crypto_begin(log_crypto_t *ctx,
                      const uint8_t recipient_public_key[LOG_CRYPTO_KEY_SIZE],
                      const uint8_t seed[LOG_CRYPTO_SEED_SIZE]);

// Returns the number of bytes written to out (record), 0 on error.
size_t log_crypto_seal(log_crypto_t *ctx, const uint8_t *plain_text,
                       size_t length, uint8_t *out, size_t out_size);

void log_crypto_end(log_crypto_t *ctx);

#endif // LOG_CRYPTO_H
