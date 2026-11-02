#ifndef SDCARD_PORT_H
#define SDCARD_PORT_H

#include "sdcard.h"

// The SPI peripheral (and SCK/MOSI pins) is shared with the OLED and must
// already be initialised by ssd1351_port_hw_init().
void sdcard_port_hw_init(void);
sdcard_io_t sdcard_port_get_io(void);

#endif // SDCARD_PORT_H
