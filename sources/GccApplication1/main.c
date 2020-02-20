/*
 * Created: 1/18/2020 11:14:42 PM
 * Motor ma 200 kroku. Puls 350+350us jen tak tak nestiha (kdyz by se predroztocil, tak by asi stihal, ale jinak ne)
 * Kdyz jsem ho roztacel pozvolna, tak se roztocil az na 25 ot/s (1500 ot/min)
 *
 * Driver provede operaci, kdyz Pulse na nabezne hrane (0 -> 1)
 *
 * Timer 0 - 1x za 1ms - pro pocitani otacek za minutu
 * Timer 1 - 
 * Timer 2 - puls pro driver
 * I2C - Display Hitachi HD44780 na adrese 0x27 (39)
 * PortB.5 = ledka primo na desce
 * PortC.2 = Driver Enable
 * PortD.3 = Driver Pulse
 * PortC.0 = Driver Direction
 * PortD.2 = zluty kabel od snimace otacek
 * PortB.2 = zeleny kabel od snimace otacek
 */ 

#include "cpu.h"
#include <avr/io.h>
#include <avr/portpins.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "main.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "buttons.h"
#include "led.h"
#include "setup_menu.h"
#include "revolutions.h"


static /*volatile*/ mode_t mode = LEFT;

static void recalculate_support_position(uint16_t current_position, uint16_t current_angle);

/******* Angle and position ******/
static /*volatile*/ uint16_t current_spindle_angle; // natoceni vretena
static /*volatile*/ uint16_t current_spindle_angle_overflow; // can overflow
static /*volatile*/ uint16_t current_spindle_absolute_position;
static /*volatile*/ uint16_t end_position = UINT16_MAX;

uint16_t get_current_spindle_angle_overflow() {
	return current_spindle_angle_overflow;
}

static void spindle_try_to_set_position_limit() {
	if (button_1_is_pressed() && (end_position == UINT16_MAX)) {
		end_position = current_spindle_absolute_position;
	}
}

static uint8_t spindle_rotate_left() {
	uint8_t direction = PINB & (1 << PD2);
	if (mode == LEFT) {
		return direction;
	} else {
		return !direction;
	}	
}

static void schedule_support_position_recalculation() {
	TIMSK2 |= 1 << OCIE2A;
}

static void spindle_position_recalculation() {
	if (spindle_rotate_left()) { // rotating left or right?
		current_spindle_angle_overflow++;
		if (current_spindle_angle < STEPS_FOR_ONE_TURN - 1) {
			current_spindle_angle++;
		} else {
			current_spindle_angle = 0;
			if (current_spindle_absolute_position < end_position) {
				current_spindle_absolute_position++;
			}
		}
	} else {
		current_spindle_angle_overflow--;
		if (current_spindle_angle > 0) {
			current_spindle_angle--;
		} else {
			current_spindle_angle = STEPS_FOR_ONE_TURN - 1;
			if (current_spindle_absolute_position > 0) {
				current_spindle_absolute_position--;
			}
		}
	}
	
	recalculate_support_position(current_spindle_absolute_position, current_spindle_angle);
	schedule_support_position_recalculation();
}
	
//Rotary Encoder interrupt
ISR(INT0_vect) { //Interrupt Vectors in ATmega328P - page 48
	spindle_position_recalculation();
}

static void init_step_counting() {
	// Phase wire 1 - External Interrupts External Interrupts - page 58
	EICRA |= (1 << ISC01) | (1 << ISC00); // INT0 on rising edge
	EIMSK |= (1 << INT0); // Enable INT0
	
	// Phase wire 2 
	DDRB &= ~(1 << PD2); // PIN as input
}

/*********** stepper-motor ***************/
static uint32_t stepper_motor_absolute_position = 0;

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

static uint8_t stepper_motor_move_was_finished() {
	return (TIFR2 == 0x07) && (TCNT2 <= 1);
}

static void stepper_motor_move_towards(uint32_t required) {
	if (stepper_motor_absolute_position < required) {
		stepper_motor_move_step_left();
		stepper_motor_absolute_position++;
	} else if (stepper_motor_absolute_position > required) {
		stepper_motor_move_step_right();
		stepper_motor_absolute_position--;
	}

	
	if (required != stepper_motor_absolute_position) {
		TIMSK2 |= 1 << OCIE2A;
	}
}

static void stepper_motor_init() {
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
	
	stepper_motor_move_step_left(); // init Timer's flags;
}

/******* support position recalculation *********/
static /*volatile*/ float step_multiplier_float = 1;
static /*volatile*/ uint32_t required_support_position = 0;

static void recalculate_support_position(uint16_t current_position, uint16_t current_angle) {
	if (current_position == 0) {
		current_angle = 0;
	} else {
		if (current_position >= end_position)  {
			current_angle = 0;
		}
		current_position--; //preskocime pozici 0, protoze tu nepouzivame pro posuv supportu (pocitame od jednicky)
	}
	
	//uint32_t requeired_support_position_tmp = ((((uint32_t)current_position) * STEPS_FOR_ONE_TURN + current_angle) * step_multiplier) / step_divisor;
	uint32_t requeired_support_position_tmp = ((uint32_t)current_position * STEPS_FOR_ONE_TURN + current_angle) * step_multiplier_float;
	
	required_support_position = requeired_support_position_tmp;
	//stepper_motor_move_towards(requeired_support_position_tmp);
}

ISR(TIMER2_COMPA_vect) {
	TIMSK2 &= ~(1 << OCIE2A); // disable interrupts
	
	uint32_t requeired_support_position_tmp;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		requeired_support_position_tmp = required_support_position;
	}
	
	stepper_motor_move_towards(requeired_support_position_tmp);
}


/****** Display information *********/
static void display_redraw() {
	char mode_char;
	if (mode == LEFT) {
		mode_char = 'L';
	} else if (mode == RIGHT) {
		mode_char = 'P';
	} else {
		mode_char = '?';
	}
	lcd_set_cursor(0, 0);
	lcd_printf("vreteno: %4u  %5u", current_spindle_angle, current_spindle_absolute_position);
	lcd_set_cursor(0, 1);
	lcd_printf("%3u/%-3u%c%5i ot/min", get_configured_multiplier(), get_configured_divisor(), mode_char, get_revolutions_per_minute());
	lcd_set_cursor(0, 2);
	lcd_printf("support: %11lu", stepper_motor_absolute_position);
	lcd_set_cursor(0, 3);
	lcd_printf("%2u %5i %11lu", button_status(), (int16_t) (required_support_position - stepper_motor_absolute_position), required_support_position);
}

static void display_init_information() {
	lcd_clear();
	lcd_disable_cursor();
	lcd_disable_blinking();
	display_redraw();
}

/************** main **************/

int main(void) {
	PORTB = 0xFF; /* enable pull up on PORTB */
	PORTC = 0xFF; /* enable pull up on PORTC */
	PORTD = 0xFF; /* enable pull up on PORTD */
		
		
		/*
		stepper_motor_init();
		while (1) {

			stepper_motor_move_step_left();
		
			_delay_us(700);
		}
		*/
		
	init_buttons();
	led_init();
	i2c_init();
	i2c_start_wait(LCD_DISPLAY_ADDRESS << 1 | I2C_WRITE);
	lcd_init();
	stepper_motor_init();
	
	user_setup_values();
	step_multiplier_float = get_configured_fraction();
	mode = get_configured_mode();
	
	display_init_information();
	
	init_step_counting();
	init_revolution_calculation();
	sei(); // enable interrupts


//PORTC &= ~(1 << PORTC2); // disable Driver!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    while (1) {
		spindle_try_to_set_position_limit();
		display_redraw();
    }
	
	/*
	DDRB |= 1 << DDB5;
	while (1) {
		PORTB |= 1 << PORTB5;
		for(uint32_t i = 0; i < 160000 ; i++)
		;
		PORTB &= ~(1 << PORTB5);
		for(uint32_t i = 0; i < 1600000 ; i++)
		;
	}
	*/
		
}


