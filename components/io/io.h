#ifndef COMPONENTS_IO_IO
#define COMPONENTS_IO_IO

#include <stdint.h>

void io_init(void);

void io_set_red_led(uint8_t state);

void io_set_blue_led(uint8_t state);

void io_set_green_led(uint8_t state);

void io_set_buzzer(uint8_t state);

void io_set_relay(uint8_t level);

uint8_t io_get_relay(void);

#endif /* COMPONENTS_IO_IO */
