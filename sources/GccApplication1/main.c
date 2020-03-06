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
#include "support.h"

static /*volatile*/ mode_t mode = LEFT;
static /*volatile*/ mode_t new_mode = LEFT;

/******* Angle and position ******/
static /*volatile*/ uint16_t end_position = UINT16_MAX;

//static /*volatile*/ uint32_t current_spindle_revolution_steps;
static /*volatile*/ uint16_t current_spindle_revolutions;
static /*volatile*/ uint16_t current_spindle_angle;
static /*volatile*/ uint16_t spindle_revolution_steps_overflow; //can overflow

uint32_t get_end_position() {
	return end_position;
}

uint16_t get_spindle_revolution_steps_overflow() {
	return spindle_revolution_steps_overflow;
}

static void spindle_try_to_set_position_limit() {
	if (button_1_is_pressed() && (end_position == UINT16_MAX)) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			end_position = current_spindle_revolutions + 1;
		}
	}
}

static void spindle_try_to_set_direction() {
	if (button_5_is_pressed()) {
		if (new_mode == LEFT) {
			new_mode = RIGHT;
		} else {
			new_mode = LEFT;
		}
		_delay_ms(500);
	}
}

static uint8_t is_spindle_rotating_left() {
	uint8_t direction = PINB & (1 << PD2);
	if (mode == LEFT) {
		return direction;
	} else {
		return !direction;
	}	
}

static void spindle_position_recalculation() {
	if (current_spindle_angle == 0) {
		mode = new_mode;
	}
	if (is_spindle_rotating_left()) { // rotating left or right?
		spindle_revolution_steps_overflow++;
		current_spindle_angle++;

		if (current_spindle_angle == STEPS_FOR_ONE_TURN) {
			current_spindle_angle = 0;
			if (current_spindle_revolutions != end_position) {
				current_spindle_revolutions++;
			}
		}
		
		if ((current_spindle_revolutions > 0) && (current_spindle_revolutions < end_position) 
			&& !((current_spindle_revolutions == 1) && (current_spindle_angle == 0))) {
			support_spindle_incremented_event();
		}
	} else {
		spindle_revolution_steps_overflow--;
		current_spindle_angle--;
		if (current_spindle_angle == UINT16_MAX) { // if overflow
			current_spindle_angle = STEPS_FOR_ONE_TURN - 1;
			if (current_spindle_revolutions != 0) {
				current_spindle_revolutions--;
			}
		}
				
		if ((current_spindle_revolutions > 0) && (current_spindle_revolutions < end_position)
			&& !((current_spindle_revolutions == (end_position - 1)) && (current_spindle_angle == (STEPS_FOR_ONE_TURN - 1)))) {
			support_spindle_decremented_event();
		}
	}
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
	
	char limit;
	if (current_spindle_revolutions == end_position) {
		limit = '>';
	} else if (current_spindle_revolutions == 0) {
		limit = '<';
	} else {
		limit = ' ';
	}
	
	lcd_set_cursor(0, 0);
	lcd_printf("vreteno: %4u  %5u", current_spindle_angle, current_spindle_revolutions);
	lcd_set_cursor(0, 1);
	lcd_printf("%3u/%-3u%c%5i ot/min", get_configured_numerator(), get_configured_denominator(), mode_char, get_revolutions_per_minute());
	lcd_set_cursor(0, 2);
	lcd_printf("support: %11lu", get_actual_support_position());
	//lcd_printf("support: %11lu", get_required_support_position());
	lcd_set_cursor(0, 3);
	lcd_printf("%2u%c%5i %11lu", button_status(), limit, (int16_t) (get_required_support_position() - get_actual_support_position()), get_required_support_position());
	/*
	if (TIFR1 & (1 << TOV1))
		lcd_printf("%2u %7i overflow", button_status(), (int16_t) (get_required_support_position() - get_actual_support_position()));
	else 
		lcd_printf("%2u %7i %9u", button_status(), (int16_t) (get_required_support_position() - get_actual_support_position()), TCNT1
	*/
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
				
	init_buttons();
	led_init();
	i2c_init();
	i2c_start_wait(LCD_DISPLAY_ADDRESS << 1 | I2C_WRITE);
	lcd_init();
	support_init();
	
	user_setup_values();
	support_set_fraction(get_configured_numerator(), get_configured_denominator());
	mode = get_configured_mode();
	
	display_init_information();
	
	init_step_counting();
	init_revolution_calculation();
	sei(); // enable interrupts


	//PORTC &= ~(1 << PORTC2); // disable Driver!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	/*
	support_set_fraction(10, 3);
	mode = RIGHT;
	TIFR1 = 0xFF;
	TCCR1B = (1 << CS12) | (1 << CS10);
	//13540   pro 30600->100000   10/3
	//20906   pro 30600,-15000->100000->50000   10/3
	//17981   pro 30600,-15000->100000->50000   10/3 ruske deleni-
	//12396   pro 30600,-15000->100000->50000   10/3 ruske deleni-+
	for(uint16_t i = 0; i<30600; i++) {
		spindle_position_recalculation();
	}
	mode = LEFT;
	for(uint16_t i = 0; i<15000; i++) {
		spindle_position_recalculation();
	}
	TCCR1B = 0;
	*/
	
    while (1)  {
		spindle_try_to_set_position_limit();
		spindle_try_to_set_direction();
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


