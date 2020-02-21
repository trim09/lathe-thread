#ifndef SETUP_MENU_H_
#define SETUP_MENU_H_

#include <stdint.h>

typedef enum {
	LEFT,
	RIGHT
} mode_t;

void user_setup_values();

float get_configured_fraction();
uint8_t get_configured_numerator();
uint8_t get_configured_denominator();

mode_t get_configured_mode();

#endif /* SETUP_MENU_H_ */