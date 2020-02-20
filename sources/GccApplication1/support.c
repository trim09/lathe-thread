#include "support.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include "main.h"

/******* support position recalculation *********/
static volatile float step_multiplier_float = 1.;
static volatile uint32_t required_support_position = 0;
static uint32_t actual_support_position = 0;

void support_set_fraction(float fraction) {
	step_multiplier_float = fraction;
}

uint32_t get_actual_support_position() {
	return actual_support_position;
}

uint32_t get_required_support_position() {
	return required_support_position;
}

void recalculate_support_position(uint32_t current_spindle_revolution_steps) {
	if (current_spindle_revolution_steps < STEPS_FOR_ONE_TURN) {
		current_spindle_revolution_steps = 0;
	} else {
		uint32_t end_position = get_end_position();
		if (current_spindle_revolution_steps > end_position) {
			current_spindle_revolution_steps = end_position;
		}
		current_spindle_revolution_steps -= STEPS_FOR_ONE_TURN; // skip the first turn
	}

	required_support_position = current_spindle_revolution_steps * step_multiplier_float;
}

/*********** stepper-motor ***************/
static void stepper_do_pulse() {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		TCNT2 = 3;
		TIFR2 = 0xFF; // clear all flags
	}
}

static void stepper_motor_move_step_left() {
	PORTC |= 1 << PORTC0;
	stepper_do_pulse();
}

static void stepper_motor_move_step_right() {
	PORTC &= ~(1 << PORTC0);
	stepper_do_pulse();
}

// static uint8_t stepper_motor_move_was_finished() {
//	return (TIFR2 == 0x07) && (TCNT2 <= 1);
//}

static void stepper_motor_move_towards(uint32_t required_support_position) {

	if (actual_support_position < required_support_position) {
		stepper_motor_move_step_left();
		actual_support_position++;
	} else if (actual_support_position > required_support_position) {
		stepper_motor_move_step_right();
		actual_support_position--;
	}

	if (required_support_position != actual_support_position) {
		TIMSK2 |= 1 << OCIE2A;
	}
}

ISR(TIMER2_COMPA_vect) {
	TIMSK2 &= ~(1 << OCIE2A); // disable interrupts
	
	uint32_t requeired_support_position_tmp;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { 
		requeired_support_position_tmp = required_support_position;
	}
	
	stepper_motor_move_towards(requeired_support_position_tmp);
}

void support_init() {
	DDRC |= 1 << DDC2; // Driver Enable
	DDRD |= 1 << DDD3; // Driver Pulse
	DDRC |= 1 << DDC0; // Driver Direction
	PORTC |= 1 << PORTC2; // Driver Enable -> 1 = enabled
		
	/* we are using this timer as a one shot timer https://hackaday.com/2015/03/24/avr-hardware-timer-tricked-into-one-shot/ */
	TCCR2A = (1 << COM2B1) | (1 << COM2B0) | (1 << WGM21) | (1 << WGM20);
	//TCCR2B = (1 << WGM22) | (1 << CS21) | (1 << CS20); // CLK / 64
	TCCR2B = (1 << WGM22) | (1 << CS21); // CLK / 8
	OCR2A = 0;
	OCR2B = 200;
	
	stepper_motor_move_step_right(); // init Timer's flags;
}
