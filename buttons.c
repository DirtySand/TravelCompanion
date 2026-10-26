#include "buttons.h"
#include "board_config.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define BUTTON_RELEASE_SETTLE_MS 50
#define BUTTON_EDGES (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)
#define BUTTON_SAMPLE_MS 10
#define BUTTON_RELEASE_SAMPLES 3

static const uint8_t button_pins[BUTTON_COUNT] = {
    [BUTTON_TOP_LEFT] = BUTTON_PIN_0,
    [BUTTON_TOP_RIGHT] = BUTTON_PIN_1,
    [BUTTON_BOTTOM_LEFT] = BUTTON_PIN_2,
    [BUTTON_BOTTOM_RIGHT] = BUTTON_PIN_3,
};

static volatile uint32_t pending_events;
static uint64_t last_edge_us[BUTTON_COUNT];
static uint32_t long_press_buttons;
static bool tracking[BUTTON_COUNT];
static uint16_t held_ms[BUTTON_COUNT];
static uint8_t release_samples[BUTTON_COUNT];
static bool long_reported[BUTTON_COUNT];

static int64_t sample_button(alarm_id_t alarm, void *user_data) {
  (void)alarm;
  uint8_t id = (uint8_t)(uintptr_t)user_data;
  if (gpio_get(button_pins[id]) == 0) {
    release_samples[id] = 0;
    if (!long_reported[id]) {
      held_ms[id] += BUTTON_SAMPLE_MS;
      if (held_ms[id] >= BUTTON_LONG_PRESS_MS) {
        long_reported[id] = true;
        pending_events |= BUTTON_LONG_MASK(id);
      }
    }
    return -(int64_t)BUTTON_SAMPLE_MS * 1000;
  }
  release_samples[id]++;
  if (release_samples[id] < BUTTON_RELEASE_SAMPLES) {
    return -(int64_t)BUTTON_SAMPLE_MS * 1000;
  }
  if (!long_reported[id]) {
    pending_events |= BUTTON_MASK(id);
  }
  tracking[id] = false;
  return 0;
}

static void start_tracking(uint8_t id) {
  held_ms[id] = 0;
  release_samples[id] = 0;
  long_reported[id] = false;
  tracking[id] = true;
  if (add_alarm_in_ms(BUTTON_SAMPLE_MS, sample_button, (void *)(uintptr_t)id,
                      true) < 0) {
    tracking[id] = false;
    pending_events |= BUTTON_MASK(id);
  }
}

static void buttons_isr(uint gpio, uint32_t events) {
  uint64_t now = time_us_64();
  for (uint8_t id = 0; id < BUTTON_COUNT; id++) {
    if (button_pins[id] != gpio) {
      continue;
    }
    bool quiet = (now - last_edge_us[id]) >= BUTTON_DEBOUNCE_US;
    last_edge_us[id] = now;
    if (!(events & GPIO_IRQ_EDGE_FALL) || !quiet) {
      return;
    }
    if (!(long_press_buttons & BUTTON_MASK(id))) {
      pending_events |= BUTTON_MASK(id);
    } else if (!tracking[id]) {
      start_tracking(id);
    }
    return;
  }
}

void buttons_init(uint32_t long_press_mask) {
  long_press_buttons = long_press_mask;
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