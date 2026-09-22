#ifndef power_h
#define power_h

#include <stdint.h>

void power_dormant_until_pins(uint8_t rtc_int_pin, uint8_t button_pin);

#endif // power_h