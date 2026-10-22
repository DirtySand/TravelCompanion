#ifndef power_h
#define power_h

#include <stdint.h>

typedef enum {
  POWER_WAKE_NONE   = 0,
  POWER_WAKE_RTC    = 1u << 0,
  POWER_WAKE_BUTTON = 1u << 1,
} power_wake_t;

uint32_t power_dormant_until_pins(uint8_t rtc_int_pin, uint8_t button_pin);

#endif // power_h