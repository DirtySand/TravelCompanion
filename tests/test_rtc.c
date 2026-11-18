#include "ds3231.h"
#include "time_sync.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t reg[0x13];
static int writes;
static int rd(void *c, uint8_t a, uint8_t r, uint8_t *d, uint8_t n) {
  (void)c;
  (void)a;
  memcpy(d, &reg[r], n);
  return n;
}
static int wr(void *c, uint8_t a, uint8_t r, const uint8_t *d, uint8_t n) {
  (void)c;
  (void)a;
  memcpy(&reg[r], d, n);
  writes++;
  return n + 1;
}
int main(void) {
  ds3231_t *d = ds3231_create(NULL, 0x68, rd, wr);
  assert(d);
  reg[0x0E] = 0x1F;
  reg[0x0F] = 0x8B;
  reg[3] = 5;
  assert(ds3231_enable_alarm1_interrupt(d, true) == DS3231_OK);
  assert(reg[0x0E] == 0x1D);
  assert(ds3231_clear_alarm_flags(d) == DS3231_OK);
  assert(reg[0x0F] == 0x88);
  assert(ds3231_set_32khz_output(d, false) == DS3231_OK);
  assert(reg[0x0F] == 0x80);
  neo6m_data_t g = {.fix_valid = true,
                    .time_valid = true,
                    .date_valid = true,
                    .hours = 23,
                    .minutes = 59,
                    .seconds = 59,
                    .day = 31,
                    .month = 12,
                    .year = 26};
  writes = 0;
  assert(time_sync_rtc_from_gps(d, &g, 26) == TIME_SYNC_OK);
  assert(writes == 2 && reg[0] == 0x59 && reg[1] == 0x59 && reg[2] == 0x23 &&
         reg[3] == 1 && reg[4] == 0x31 && reg[5] == 0x12 && reg[6] == 0x26 &&
         reg[0x0F] == 0x00);
  ds3231_datetime_t t;
  assert(ds3231_read_datetime(d, &t) == DS3231_OK && t.hours == 23 &&
         t.day == 31 && t.year == 26);
  g.year = 19;
  assert(time_sync_rtc_from_gps(d, &g, 26) == TIME_SYNC_IMPLAUSIBLE_DATE);
  g.year = 26;
  g.seconds = 60;
  assert(time_sync_rtc_from_gps(d, &g, 26) == TIME_SYNC_LEAP_SECOND);
  puts("rtc ok");
  return 0;
}
