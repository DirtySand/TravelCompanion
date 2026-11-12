#include "power.h"
#include "board_config.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/sync.h"
#include "hardware/xosc.h"
#include "pico/sleep.h"
#include "pico/stdlib.h"

#define GPIO_COUNT 30u

static bool dormant_wake_pending(uint8_t pin, uint32_t event) {
  uint32_t mask = event << (4u * (pin % 8u));
  return (io_bank0_hw->dormant_wake_irq_ctrl.ints[pin / 8u] & mask) != 0;
}

static void set_wake_sources(uint8_t rtc_int_pin, uint32_t button_pin_mask,
                             bool enabled) {
  gpio_set_dormant_irq_enabled(rtc_int_pin, GPIO_IRQ_LEVEL_LOW, enabled);
  for (uint8_t pin = 0; pin < GPIO_COUNT; pin++) {
    if (button_pin_mask & POWER_PIN_MASK(pin)) {
      gpio_set_dormant_irq_enabled(pin, GPIO_IRQ_EDGE_FALL, enabled);
    }
  }
}

uint32_t power_dormant_until(uint8_t rtc_int_pin, uint32_t button_pin_mask) {
  uint32_t irq_state = save_and_disable_interrupts();

  sleep_run_from_xosc();
  set_wake_sources(rtc_int_pin, button_pin_mask, true);

  xosc_dormant();

  uint32_t wake = 0;
  if (dormant_wake_pending(rtc_int_pin, GPIO_IRQ_LEVEL_LOW)) {
    wake |= POWER_PIN_MASK(rtc_int_pin);
  }
  for (uint8_t pin = 0; pin < GPIO_COUNT; pin++) {
    if ((button_pin_mask & POWER_PIN_MASK(pin)) &&
        dormant_wake_pending(pin, GPIO_IRQ_EDGE_FALL)) {
      wake |= POWER_PIN_MASK(pin);
    }
  }

  set_wake_sources(rtc_int_pin, button_pin_mask, false);
  for (uint8_t pin = 0; pin < GPIO_COUNT; pin++) {
    if (button_pin_mask & POWER_PIN_MASK(pin)) {
      gpio_acknowledge_irq(pin, GPIO_IRQ_EDGE_FALL);
    }
  }

  sleep_power_up();
  set_sys_clock_khz(SYS_CLOCK_KHZ, true);
  restore_interrupts(irq_state);
  return wake;
}
