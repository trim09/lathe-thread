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
#include "i2cmaster.h"
#include "lcd.h"

#define LCD_DISPLAY_ADDRESS 0x27

#define SUPPORT_RECALCULATION_SPEED 128 // 16 MHz / 64 / 128 / 2 ~ 1 kHz   deleno 2 protoze v jednom kroku nastavime puls na Driveru na 1 a pak v druhem na 0

#define STEPS_FOR_ONE_TURN 600u

typedef enum {
	LEFT,
	RIGHT
} mode_t;

static /*volatile*/ mode_t mode = LEFT;

static void recalculate_support_position(uint16_t current_position, uint16_t current_angle);

/******* On board led ******/
static void led_init() {
	DDRB |= 1 << DDB5;
}

static void led_off() {
	PORTB &= ~(1 << PORTB5);
}

static void led_on() {
	PORTB |= 1 << PORTB5;
}

static void led_toggle() {
	PORTB ^= 1 << PORTB5;
}

/******* buttons *********/
static void init_buttons() {
	DDRD &= ~(1 << DDD4);
	DDRD &= ~(1 << DDD6);
	DDRD &= ~(1 << DDD7);
	DDRB &= ~(1 << DDB0);
	DDRB &= ~(1 << DDB1);
}

static uint8_t button_1_is_pressed() {
	return !(PIND & (1 << PIND4));
}

static uint8_t button_2_is_pressed() {
	return !(PIND & (1 << PIND6));
}

static uint8_t button_3_is_pressed() {
	return !(PIND & (1 << PIND7));
}

static uint8_t button_4_is_pressed() {
	return !(PINB & (1 << PINB0));
}

static uint8_t button_5_is_pressed() {
	return !(PINB & (1 << PINB1));
}

static uint8_t button_status() {
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

/******* Angle and position ******/
static /*volatile*/ uint16_t current_spindle_angle; // natoceni vretena
static /*volatile*/ uint16_t current_spindle_angle_overflow; // can overflow
static /*volatile*/ uint16_t current_spindle_absolute_position;
static /*volatile*/ uint16_t end_position = UINT16_MAX;

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
static /*volatile*/ uint8_t step_multiplier = 1u;
static /*volatile*/ uint8_t step_divisor = 1u;
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


/********* revolutions per second calculation **************/
static /*volatile*/ int16_t spindle_revolutions_per_minute = 0;
static uint16_t previous_spindle_revolutions = 0;
static uint16_t one_ms_to_one_second = 0;

static void init_revolution_calculation(void) {
	OCR0A = 125u; // 16 MHz / 256 / 125 = 2 ms
	TCCR0A = 1 << WGM01; // Clear Timer on Compare Match (CTC) Mode
	TCCR0B = 1 << CS02; // CLK / 256x
	TIMSK0 = 1 << OCIE0A; // enable interrupt
}

/* call this method once per second */
static void recalculate_revolutions_per_second() {
	uint16_t tmp = current_spindle_angle_overflow;
	int16_t angle_increment_per_second = tmp - previous_spindle_revolutions;
	previous_spindle_revolutions = tmp;
	
	int32_t angle_increment_per_minute = 60l * angle_increment_per_second;
	spindle_revolutions_per_minute = angle_increment_per_minute / STEPS_FOR_ONE_TURN;
}

ISR(TIMER0_COMPA_vect) { // once per 2ms
	if (one_ms_to_one_second++ == 500u) {
		one_ms_to_one_second = 0; // once per second
		recalculate_revolutions_per_second();

		led_toggle();
	}
}

/* Display information */
static void display_user_setting_values() {
	lcd_set_cursor(0, 0);
	lcd_enable_cursor();
	lcd_enable_blinking();
	lcd_printf("%-5s %03u/%03u", (mode == LEFT) ? "Levy" : "Pravy", step_multiplier, step_divisor);
	lcd_set_cursor(0, 1);
	lcd_printf("%f", step_multiplier_float);
}

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
	lcd_printf("%3u/%-3u%c%5i ot/min", step_multiplier, step_divisor, mode_char, spindle_revolutions_per_minute);
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

/************* user setup values / menu **************/
static uint8_t user_add_witout_overflow(uint8_t orig, int8_t increment) {
	int16_t res = orig;
	res += increment;
	if (res > UINT8_MAX) {
		return UINT8_MAX;
	} else if (res < 0) {
		return 0;
	} else {
		return res;
	}
}

static uint8_t user_setup_next_position(uint8_t prev) {
	switch (prev) {
		case 0: return 6;
		case 6: return 7;
		case 7: return 8;
		case 8: return 10;
		case 10: return 11;
		case 11: return 12;
		case 12: return UINT8_MAX;
		default: return UINT8_MAX;
	}
}

static void user_setup_values() {
	uint8_t position = 0;
	while(position != UINT8_MAX) {
		display_user_setting_values();
		lcd_set_cursor(position, 0);	
		
		if (button_1_is_pressed()) {
			position = user_setup_next_position(position);
		} else {
			switch(position) {
				case 0:
					if (button_2_is_pressed()) {
						if (mode == LEFT) {
							mode = RIGHT;
						} else if (mode == RIGHT) {
							mode = LEFT;
						}
					}
					break;
				case 6:	
					if (button_2_is_pressed()) {
						step_multiplier = user_add_witout_overflow(step_multiplier, 100);
					} else if (button_3_is_pressed()) {
						step_multiplier = user_add_witout_overflow(step_multiplier, -100);
					}
					break;
				case 7:
					if (button_2_is_pressed()) {
						step_multiplier = user_add_witout_overflow(step_multiplier, 10);
					} else if (button_3_is_pressed()) {
						step_multiplier = user_add_witout_overflow(step_multiplier, -10);
					}
					break;
				case 8:
					if (button_2_is_pressed()) {
						step_multiplier = user_add_witout_overflow(step_multiplier, 1);
					} else if (button_3_is_pressed()) {
						step_multiplier = user_add_witout_overflow(step_multiplier, -1);
					}
						break;
				case 10:
					if (button_2_is_pressed()) {
						step_divisor = user_add_witout_overflow(step_divisor, 100);
					} else if (button_3_is_pressed()) {
						step_divisor = user_add_witout_overflow(step_divisor, -100);
					}
					break;
				case 11:
					if (button_2_is_pressed()) {
						step_divisor = user_add_witout_overflow(step_divisor, 10);
					} else if (button_3_is_pressed()) {
						step_divisor = user_add_witout_overflow(step_divisor, -10);
					}
					break;
				case 12:
					if (button_2_is_pressed()) {
						step_divisor = user_add_witout_overflow(step_divisor, 1);
					} else if (button_3_is_pressed()) {
						step_divisor = user_add_witout_overflow(step_divisor, -1);
					}
					break;
				default:
					break;
			}	
		}
		
		if (step_divisor == 0) {
			step_divisor = 1;	
		}
		
		step_multiplier_float = (float)step_multiplier / step_divisor;
		
		while(button_status())
			;
		_delay_ms(100);
	}
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


