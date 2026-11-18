#include "bme680.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t reg[256];
static int busy_polls, delays_ms, fail_read_at = -1, reads, triggers;
static int rd(void *c, uint8_t a, uint8_t r, uint8_t *d, uint8_t n) {
  (void)c;
  (void)a;
  if (reads++ == fail_read_at)
    return -1;
  if (r == 0x1D) {
    d[0] = busy_polls > 0 ? 0x20 : 0x80;
    if (busy_polls > 0)
      busy_polls--;
    return 1;
  }
  memcpy(d, &reg[r], n);
  return n;
}
static int wr(void *c, uint8_t a, uint8_t r, const uint8_t *d, uint8_t n) {
  (void)c;
  (void)a;
  (void)n;
  reg[r] = d[0];
  if (r == 0x74 && (d[0] & 3) == 1)
    triggers++;
  return n + 1;
}
static int wr_fail(void *c, uint8_t a, uint8_t r, const uint8_t *d, uint8_t n) {
  (void)c;
  (void)a;
  (void)r;
  (void)d;
  (void)n;
  return -1;
}
static void dl(void *c, uint32_t ms) {
  (void)c;
  delays_ms += ms;
}
int main(void) {
  reg[0xE9] = 0xB9;
  reg[0xEA] = 0x65;
  reg[0x8A] = 0x65;
  reg[0x8B] = 0x67;
  reg[0x8C] = 3;
  reg[0x8E] = 0xA6;
  reg[0x8F] = 0x8D;
  reg[0x90] = 0x7D;
  reg[0x91] = 0xD7;
  reg[0x92] = 88;
  reg[0x94] = 0x8E;
  reg[0x95] = 0x1C;
  reg[0x96] = 0x7F;
  reg[0x97] = 0xFF;
  reg[0x99] = 30;
  reg[0x98] = 37;
  reg[0x9C] = 0x1D;
  reg[0x9D] = 0xFC;
  reg[0x9E] = 0x8F;
  reg[0x9F] = 0xF3;
  reg[0xA0] = 30;
  assert(bme680_create(NULL, 0x77, rd, wr_fail, dl) == NULL);
  bme680_t *d = bme680_create(NULL, 0x77, rd, wr, dl);
  assert(d);
  reg[0x1F] = 0x55;
  reg[0x20] = 0x73;
  reg[0x21] = 0x00;
  reg[0x22] = 0x7A;
  reg[0x23] = 0x12;
  reg[0x24] = 0x00;
  reg[0x25] = 0x4E;
  reg[0x26] = 0x20;
  float t, p, h;
  busy_polls = 3;
  delays_ms = 0;
  triggers = 0;
  assert(bme680_read_measurement_forced(d, &t, &p, &h) == BME680_OK);
  printf("T=%.2f P=%.1f H=%.1f delays=%d ms triggers=%d\n", t, p, h, delays_ms,
         triggers);
  assert(triggers == 1 && delays_ms == 5 + 3 * 2 && busy_polls == 0);
  busy_polls = 1000;
  delays_ms = 0;
  assert(bme680_read_measurement_forced(d, &t, &p, &h) == BME680_TIMEOUT);
  printf("timeout after %d ms\n", delays_ms);
  busy_polls = 0;
  reads = 0;
  fail_read_at = 1;
  assert(bme680_read_measurement_forced(d, &t, &p, &h) == BME680_I2C_ERROR);
  reads = 0;
  fail_read_at = 0;
  assert(bme680_read_measurement_forced(d, &t, &p, &h) == BME680_I2C_ERROR);
  puts("bme680 ok");
  return 0;
}
