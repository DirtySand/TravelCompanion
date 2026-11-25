#include "csv.h"
#include <stdarg.h>
#include <stdio.h>

void csv_init(csv_t *csv, char *out, size_t size) {
  *csv = (csv_t){.out = out, .size = size};
  if (size > 0) {
    out[0] = '\0';
  }
}

void csv_append(csv_t *csv, const char *format, ...) {
  if (csv->overflow) {
    return;
  }
  va_list args;
  va_start(args, format);
  int written =
      vsnprintf(csv->out + csv->length, csv->size - csv->length, format, args);
  va_end(args);
  if (written < 0 || (size_t)written >= csv->size - csv->length) {
    csv->overflow = true;
    return;
  }
  csv->length += (size_t)written;
}

void csv_scaled(csv_t *csv, int32_t value, uint8_t decimals) {
  uint32_t divisor = 1;
  for (uint8_t i = 0; i < decimals; i++) {
    divisor *= 10u;
  }
  uint32_t magnitude =
      (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
  if (decimals == 0) {
    csv_append(csv, ",%s%lu", (value < 0) ? "-" : "", (unsigned long)magnitude);
    return;
  }
  csv_append(csv, ",%s%lu.%0*lu", (value < 0) ? "-" : "",
             (unsigned long)(magnitude / divisor), (int)decimals,
             (unsigned long)(magnitude % divisor));
}

void csv_float(csv_t *csv, bool valid, float value, uint8_t decimals) {
  if (!valid) {
    csv_append(csv, ",");
    return;
  }
  csv_append(csv, ",%.*f", (int)decimals, (double)value);
}

void csv_degrees(csv_t *csv, bool valid, int32_t value_e7) {
  if (!valid) {
    csv_append(csv, ",");
    return;
  }
  csv_scaled(csv, value_e7, 7);
}

size_t csv_length(const csv_t *csv) { return csv->overflow ? 0 : csv->length; }
