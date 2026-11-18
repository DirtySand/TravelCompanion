#include "datetime_utils.h"
#include <assert.h>
#include <stdio.h>

#define BASE 2000

int main(void) {
  ds3231_datetime_t t;
  datetime_from_seconds(0, BASE, &t);
  assert(t.year == 0 && t.month == 1 && t.day == 1 && t.hours == 0);

  const ds3231_datetime_t leap_day = {.seconds = 59,
                                      .minutes = 59,
                                      .hours = 23,
                                      .day = 29,
                                      .month = 2,
                                      .year = 28};
  uint32_t s = datetime_to_seconds(&leap_day, BASE);
  datetime_from_seconds(s, BASE, &t);
  assert(t.year == 28 && t.month == 2 && t.day == 29 && t.hours == 23 &&
         t.minutes == 59 && t.seconds == 59);
  datetime_from_seconds(s + 1, BASE, &t);
  assert(t.month == 3 && t.day == 1 && t.hours == 0);

  for (uint32_t x = 0; x < 3000000000u; x += 86399u * 7u) {
    datetime_from_seconds(x, BASE, &t);
    assert(datetime_to_seconds(&t, BASE) == x);
  }

  ds3231_datetime_t new_year = {.seconds = 55,
                                .minutes = 59,
                                .hours = 23,
                                .day = 31,
                                .month = 12,
                                .year = 26};
  datetime_add_seconds(&new_year, 10, BASE);
  assert(new_year.year == 27 && new_year.month == 1 && new_year.day == 1 &&
         new_year.seconds == 5);

  const ds3231_datetime_t summer = {
      .hours = 10, .day = 1, .month = 7, .year = 26};
  datetime_to_local(&summer, &t, BASE);
  assert(t.hours == 12);
  const ds3231_datetime_t winter = {
      .hours = 10, .day = 1, .month = 1, .year = 26};
  datetime_to_local(&winter, &t, BASE);
  assert(t.hours == 11);

  puts("datetime ok");
  return 0;
}
