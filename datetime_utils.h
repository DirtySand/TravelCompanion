#ifndef datetime_utils_h
#define datetime_utils_h
#include <stdint.h>

#include "drivers/ds3231/ds3231.h"

void datetime_add_seconds(ds3231_datetime_t *datetime, int seconds,
                          uint16_t century_base);

void datetime_to_local(const ds3231_datetime_t *datetime, ds3231_datetime_t *local_datetime, uint16_t century_base);

#endif // datetime_utils_h