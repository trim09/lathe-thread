#include "support.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <stdlib.h>
#include "rdivfun.h"

/******* support position recalculation *********/
//static volatile float step_multiplier_float = 1.;
static volatile uint8_t step_numerator = 1;
static volatile uint8_t step_denominator = 1;

static volatile uint32_t required_support_position = 0;
static uint32_t actual_support_position = 0;

static int16_t spindle_to_support_recalculation = 0;

void support_set_fraction(uint8_t numerator, uint8_t denominator) {
	step_numerator = numerator;
	step_denominator = denominator;
}

//void support_set_fraction(float fraction) {
//	step_multiplier_float = fraction;
//}

uint32_t get_actual_support_position() {
	return actual_support_position;
}

uint32_t get_required_support_position() {
	return required_support_position;
}

static void support_schedule_step() {
	TIMSK2 |= 1 << OCIE2A;
}

void support_spindle_incremented_event() {
	spindle_to_support_recalculation += step_numerator;
	
	if (spindle_to_support_recalculation <= 0) {
		return;
	}
	
	uint16_t quot = 0;	
	rdiv16u(&quot, (uint16_t*)&spindle_to_support_recalculation, spindle_to_support_recalculation, (uint16_t)step_denominator);
	required_support_position += (uint8_t)quot;
	
	support_schedule_step();
}

void support_spindle_decremented_event() {
	spindle_to_support_recalculation -= step_numerator;
	
	if (spindle_to_support_recalculation >= 0) {
		return;
	}
	
	uint16_t quot = 0;
	uint16_t rem = 0;
	rdiv16u(&quot, &rem, (uint16_t)-spindle_to_support_recalculation, (uint16_t)step_denominator);
		
	spindle_to_support_recalculation = -(int16_t)rem;
	required_support_position -= (uint8_t)quot;
	
	support_schedule_step();
}

/*********** stepper-motor ***************/
static void stepper_do_pulse() {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		TCNT2 = 199;
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
		support_schedule_step();
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
