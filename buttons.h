#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  BUTTON_TOP_LEFT,
  BUTTON_TOP_RIGHT,
  BUTTON_BOTTOM_LEFT,
  BUTTON_BOTTOM_RIGHT,
  BUTTON_COUNT,
} button_id_t;

#define BUTTON_MASK(id) (1u << (id))
#define BUTTON_LONG_MASK(id) (1u << ((id) + BUTTON_COUNT))

void buttons_init(uint32_t long_press_mask);
uint32_t buttons_take_events(void);
bool buttons_has_events(void);
void buttons_wait_release(button_id_t id);
uint8_t buttons_pin(button_id_t id);

#endif // BUTTONS_H