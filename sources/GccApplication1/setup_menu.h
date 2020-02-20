#ifndef SETUP_MENU_H_
#define SETUP_MENU_H_

#include <stdint.h>

typedef enum {
	LEFT,
	RIGHT
} mode_t;

void user_setup_values();

float get_configured_fraction();
mode_t get_configured_mode();
uint8_t get_configured_multiplier();
uint8_t get_configured_divisor();

#endif /* SETUP_MENU_H_ */