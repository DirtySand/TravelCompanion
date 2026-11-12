#ifndef WP_FLASH_H
#define WP_FLASH_H

#include "waypoints.h"
#include <stdbool.h>

#define WP_FLASH_REGION_SIZE (128u * 1024u)
#define WP_FLASH_SLOTS (WP_FLASH_REGION_SIZE / WAYPOINTS_SLOT_SIZE)

bool wp_flash_storage(wp_storage_t *storage);

#endif // WP_FLASH_H
