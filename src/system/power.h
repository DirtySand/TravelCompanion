#ifndef POWER_H
#define POWER_H

#include <stdbool.h>
#include <stdint.h>

#define POWER_PIN_MASK(pin) (1u << (pin))

uint32_t power_dormant_until(uint8_t rtc_int_pin, uint32_t button_pin_mask);

void power_watchdog_start(void);
void power_watchdog_kick(void);
bool power_watchdog_caused_reboot(void);

#endif // POWER_H
