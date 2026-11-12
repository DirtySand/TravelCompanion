#ifndef DATETIME_UTILS_H
#define DATETIME_UTILS_H
#include <stdint.h>

#include "ds3231.h"

void datetime_add_seconds(ds3231_datetime_t *datetime, int seconds,
                          uint16_t century_base);

void datetime_to_local(const ds3231_datetime_t *datetime,
                       ds3231_datetime_t *local_datetime,
                       uint16_t century_base);

uint32_t datetime_to_seconds(const ds3231_datetime_t *datetime,
                             uint16_t century_base);

#endif // DATETIME_UTILS_H
