#ifndef POWER_H
#define POWER_H

#include <stdint.h>

#define POWER_PIN_MASK(pin) (1u << (pin))

// Enters DORMANT until the RTC INT pin is low (level: an alarm that fired
// just before sleeping still wakes immediately) or one of the button pins
// sees a falling edge. Returns the GPIO mask of the pins that woke the chip.
uint32_t power_dormant_until(uint8_t rtc_int_pin, uint32_t button_pin_mask);

#endif // POWER_H
