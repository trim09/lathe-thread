#include "led.h"
#include <avr/io.h>

/******* On board led ******/

void led_init() {
	DDRB |= 1 << DDB5;
}

void led_off() {
	PORTB &= ~(1 << PORTB5);
}

void led_on() {
	PORTB |= 1 << PORTB5;
}

void led_toggle() {
	PORTB ^= 1 << PORTB5;
}
