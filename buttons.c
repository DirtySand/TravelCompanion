#include "buttons.h"
#include "board_config.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define BUTTON_RELEASE_SETTLE_MS 50
#define BUTTON_EDGES (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)

static const uint8_t button_pins[BUTTON_COUNT] = {
    [BUTTON_TOP_LEFT] = BUTTON_PIN_0,
    [BUTTON_TOP_RIGHT] = BUTTON_PIN_1,
    [BUTTON_BOTTOM_LEFT] = BUTTON_PIN_2,
    [BUTTON_BOTTOM_RIGHT] = BUTTON_PIN_3,
};

static volatile uint32_t pending_events;
static uint64_t last_edge_us[BUTTON_COUNT];

static void buttons_isr(uint gpio, uint32_t events) {
  uint64_t now = time_us_64();
  for (uint8_t id = 0; id < BUTTON_COUNT; id++) {
    if (button_pins[id] != gpio) {
      continue;
    }
    bool quiet = (now - last_edge_us[id]) >= BUTTON_DEBOUNCE_US;
    last_edge_us[id] = now;
    if ((events & GPIO_IRQ_EDGE_FALL) && quiet) {
      pending_events |= BUTTON_MASK(id);
    }
    return;
  }
}

void buttons_init(void) {
  for (uint8_t id = 0; id < BUTTON_COUNT; id++) {
    uint8_t pin = button_pins[id];
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
  }
  for (uint8_t id = 0; id < BUTTON_COUNT; id++) {
    gpio_set_irq_enabled_with_callback(button_pins[id], BUTTON_EDGES, true,
                                       &buttons_isr);
  }
}

uint32_t buttons_take_events(void) {
  uint32_t irq_state = save_and_disable_interrupts();
  uint32_t events = pending_events;
  pending_events = 0;
  restore_interrupts(irq_state);
  return events;
}

bool buttons_has_events(void) { return pending_events != 0; }

void buttons_wait_release(button_id_t id) {
  while (gpio_get(button_pins[id]) == 0) {
    tight_loop_contents();
  }
  sleep_ms(BUTTON_RELEASE_SETTLE_MS);
  buttons_take_events();
}

uint8_t buttons_pin(button_id_t id) { return button_pins[id]; }