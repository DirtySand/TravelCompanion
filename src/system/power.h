#ifndef POWER_H
#define POWER_H

#include <stdint.h>

#define POWER_PIN_MASK(pin) (1u << (pin))

uint32_t power_dormant_until(uint8_t rtc_int_pin, uint32_t button_pin_mask);

#endif // POWER_H
