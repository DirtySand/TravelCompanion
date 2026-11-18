#include "time_sync.h"
#include <stddef.h>

time_sync_status_t time_sync_rtc_from_gps(ds3231_t *rtc,
                                          const neo6m_data_t *gps,
                                          uint8_t min_year) {
  if (rtc == NULL || gps == NULL) {
    return TIME_SYNC_NULL_PTR;
  }
  if (!gps->fix_valid || !gps->time_valid || !gps->date_valid) {
    return TIME_SYNC_NO_FIX;
  }
  if (gps->seconds > 59) {
    return TIME_SYNC_LEAP_SECOND;
  }
  if (gps->year < min_year) {
    return TIME_SYNC_IMPLAUSIBLE_DATE;
  }
  const ds3231_datetime_t utc = {
      .seconds = gps->seconds,
      .minutes = gps->minutes,
      .hours = gps->hours,
      .day = gps->day,
      .month = gps->month,
      .year = gps->year,
  };
  if (ds3231_set_datetime(rtc, &utc) != DS3231_OK) {
    return TIME_SYNC_RTC_ERROR;
  }
  return TIME_SYNC_OK;
}
