#include "power.h"
#include "hardware/gpio.h"
#include "hardware/structs/io_bank0.h"
#include "pico/sleep.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "board_config.h"

void power_dormant_until_pins(uint8_t rtc_int_pin, uint8_t button_pin) {
  sleep_run_from_xosc();
  gpio_set_dormant_irq_enabled(
      button_pin, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS, true);
  sleep_goto_dormant_until_pin(rtc_int_pin, true, false);
  gpio_acknowledge_irq(button_pin,
                       IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS);
  gpio_set_dormant_irq_enabled(
      button_pin, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS, false);
  sleep_power_up();
  set_sys_clock_khz(SYS_CLOCK_KHZ, true);
}