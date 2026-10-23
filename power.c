#include "power.h"
#include "board_config.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/sync.h"
#include "hardware/xosc.h"
#include "pico/sleep.h"
#include "pico/stdlib.h"

static bool dormant_wake_pending(uint8_t pin, uint32_t event) {
  uint32_t mask = event << (4u * (pin % 8u));
  return (io_bank0_hw->dormant_wake_irq_ctrl.ints[pin / 8u] & mask) != 0;
}

uint32_t power_dormant_until_pins(uint8_t rtc_int_pin, uint8_t button_pin) {
  const uint32_t ev = GPIO_IRQ_EDGE_FALL;

  uint32_t irq_state = save_and_disable_interrupts();

  sleep_run_from_xosc();
  gpio_set_dormant_irq_enabled(rtc_int_pin, ev, true);
  gpio_set_dormant_irq_enabled(button_pin, ev, true);

  xosc_dormant();

  uint32_t wake = POWER_WAKE_NONE;
  if (dormant_wake_pending(rtc_int_pin, ev))
    wake |= POWER_WAKE_RTC;
  if (dormant_wake_pending(button_pin, ev))
    wake |= POWER_WAKE_BUTTON;

  gpio_set_dormant_irq_enabled(rtc_int_pin, ev, false);
  gpio_set_dormant_irq_enabled(button_pin, ev, false);

  sleep_power_up();
  set_sys_clock_khz(SYS_CLOCK_KHZ, true);
  restore_interrupts(irq_state);
  return wake;
}