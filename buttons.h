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

void buttons_init(void);
uint32_t buttons_take_events(void);
bool buttons_has_events(void);
void buttons_wait_release(button_id_t id);
uint8_t buttons_pin(button_id_t id);

#endif // BUTTONS_H