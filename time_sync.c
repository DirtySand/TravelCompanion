#include "time_sync.h"
#include <stddef.h>

time_sync_status_t time_sync_rtc_from_gps(ds3231_t *rtc,
                                          const neo6m_data_t *gps) {
  if (rtc == NULL || gps == NULL) {
    return TIME_SYNC_NULL_PTR;
  }
  if (!gps->fix_valid || !gps->time_valid || !gps->date_valid) {
    return TIME_SYNC_NO_FIX;
  }
  if (gps->seconds > 59) {
    return TIME_SYNC_LEAP_SECOND;
  }
  if (ds3231_set_time(rtc, gps->hours, gps->minutes, gps->seconds) !=
      DS3231_OK) {
    return TIME_SYNC_RTC_ERROR;
  }
  if (ds3231_set_date(rtc, gps->day, gps->month, gps->year) != DS3231_OK) {
    return TIME_SYNC_RTC_ERROR;
  }
  return TIME_SYNC_OK;
}