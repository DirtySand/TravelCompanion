#ifndef CSV_H
#define CSV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  char *out;
  size_t size;
  size_t length;
  bool overflow;
} csv_t;

void csv_init(csv_t *csv, char *out, size_t size);
void csv_append(csv_t *csv, const char *format, ...)
    __attribute__((format(printf, 2, 3)));

void csv_scaled(csv_t *csv, int32_t value, uint8_t decimals);

void csv_float(csv_t *csv, bool valid, float value, uint8_t decimals);

void csv_degrees(csv_t *csv, bool valid, int32_t value_e7);

size_t csv_length(const csv_t *csv);

#endif // CSV_H
