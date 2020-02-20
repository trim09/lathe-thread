#include "revolutions.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include "led.h"
#include "main.h"

/********* revolutions per second calculation **************/
static /*volatile*/ int16_t spindle_revolutions_per_minute = 0;
static uint16_t previous_spindle_revolutions = 0;
static uint16_t x_ms_to_one_second = 0;

int16_t get_revolutions_per_minute() {
	return spindle_revolutions_per_minute;
}

void init_revolution_calculation(void) {
	OCR0A = 125u; // 16 MHz / 256 / 125 = 2 ms
	TCCR0A = 1 << WGM01; // Clear Timer on Compare Match (CTC) Mode
	TCCR0B = 1 << CS02; // CLK / 256x
	TIMSK0 = 1 << OCIE0A; // enable interrupt
}

/* call this method once per second */
static void recalculate_revolutions_per_second() {
	uint16_t tmp = get_spindle_revolution_steps_overflow();
	int16_t angle_increment_per_second = tmp - previous_spindle_revolutions;
	previous_spindle_revolutions = tmp;
	
	int32_t angle_increment_per_minute = 60l * angle_increment_per_second;
	spindle_revolutions_per_minute = angle_increment_per_minute / STEPS_FOR_ONE_TURN;
}

ISR(TIMER0_COMPA_vect) { // once per 2ms
	if (x_ms_to_one_second++ == 500u) {
		x_ms_to_one_second = 0; // once per second
		recalculate_revolutions_per_second();

		led_toggle();
	}
}