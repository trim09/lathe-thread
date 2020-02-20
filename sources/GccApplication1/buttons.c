#include <avr/io.h>
#include "buttons.h"

void init_buttons() {
	DDRD &= ~(1 << DDD4);
	DDRD &= ~(1 << DDD6);
	DDRD &= ~(1 << DDD7);
	DDRB &= ~(1 << DDB0);
	DDRB &= ~(1 << DDB1);
}

uint8_t button_1_is_pressed() {
	return !(PIND & (1 << PIND4));
}

uint8_t button_2_is_pressed() {
	return !(PIND & (1 << PIND6));
}

uint8_t button_3_is_pressed() {
	return !(PIND & (1 << PIND7));
}

uint8_t button_4_is_pressed() {
	return !(PINB & (1 << PINB0));
}

uint8_t button_5_is_pressed() {
	return !(PINB & (1 << PINB1));
}

uint8_t button_status() {
	uint8_t ret = 0;
	if (button_1_is_pressed()) {
		ret |= 1;
	}
	if (button_2_is_pressed()) {
		ret |= 1 << 1;
	}
	if (button_3_is_pressed()) {
		ret |= 1 << 2;
	}
	if (button_4_is_pressed()) {
		ret |= 1 << 3;
	}
	if (button_5_is_pressed()) {
		ret |= 1 << 4;
	}
	return ret;
}
