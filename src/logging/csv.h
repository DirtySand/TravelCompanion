#ifndef CSV_H
#define CSV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Bounded CSV line builder. After the first write that does not fit, every
// further call is ignored and csv_length() returns 0.

typedef struct {
  char *out;
  size_t size;
  size_t length;
  bool overflow;
} csv_t;

void csv_init(csv_t *csv, char *out, size_t size);
void csv_append(csv_t *csv, const char *format, ...)
    __attribute__((format(printf, 2, 3)));

// ",<value / 10^decimals>" with exactly `decimals` digits after the point,
// e.g. csv_scaled(csv, -1234, 2) -> ",-12.34". Integer maths only.
void csv_scaled(csv_t *csv, int32_t value, uint8_t decimals);

// ",<value>" with `decimals` digits, or "," when !valid.
void csv_float(csv_t *csv, bool valid, float value, uint8_t decimals);

// "," when !valid, else csv_scaled(value_e7, 7): degrees from 1e-7 units.
void csv_degrees(csv_t *csv, bool valid, int32_t value_e7);

size_t csv_length(const csv_t *csv);

#endif // CSV_H
