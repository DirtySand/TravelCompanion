#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include "drivers/ds3231/ds3231.h"
#include "drivers/neo6m/neo6m.h"

typedef enum {
  TIME_SYNC_OK,
  TIME_SYNC_NO_FIX,
  TIME_SYNC_LEAP_SECOND,
  TIME_SYNC_RTC_ERROR,
  TIME_SYNC_NULL_PTR,
} time_sync_status_t;

time_sync_status_t time_sync_rtc_from_gps(ds3231_t *rtc,
                                          const neo6m_data_t *gps);

#endif // TIME_SYNC_H