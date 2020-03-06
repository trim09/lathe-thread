#include "setup_menu.h"
#include "cpu.h"
#include <util/delay.h>
#include <avr/io.h>
#include "buttons.h"
#include "lcd.h"

static uint8_t step_numerator = 128u;
static uint8_t step_denominator = 128u;
static mode_t mode = LEFT;

float get_configured_fraction() {
	return (float)step_numerator / step_denominator;
}

mode_t get_configured_mode() {
	return mode;
}

uint8_t get_configured_numerator() {
	return step_numerator;
}

uint8_t get_configured_denominator() {
	return step_denominator;
}

static void display_user_setting_values() {
	lcd_set_cursor(0, 0);
	lcd_enable_cursor();
	lcd_enable_blinking();
	lcd_printf("%-5s %03u/%03u", (mode == LEFT) ? "Levy" : "Pravy", step_numerator, step_denominator);
	lcd_set_cursor(0, 2);
	lcd_printf("%03u/%03u = %f", step_numerator, step_denominator, get_configured_fraction());
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

void user_setup_values() {
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
						step_numerator = user_add_witout_overflow(step_numerator, 100);
					} else if (button_3_is_pressed()) {
						step_numerator = user_add_witout_overflow(step_numerator, -100);
					}
					break;
				case 7:
					if (button_2_is_pressed()) {
						step_numerator = user_add_witout_overflow(step_numerator, 10);
					} else if (button_3_is_pressed()) {
						step_numerator = user_add_witout_overflow(step_numerator, -10);
					}
					break;
				case 8:
					if (button_2_is_pressed()) {
						step_numerator = user_add_witout_overflow(step_numerator, 1);
					} else if (button_3_is_pressed()) {
						step_numerator = user_add_witout_overflow(step_numerator, -1);
					}
						break;
				case 10:
					if (button_2_is_pressed()) {
						step_denominator = user_add_witout_overflow(step_denominator, 100);
					} else if (button_3_is_pressed()) {
						step_denominator = user_add_witout_overflow(step_denominator, -100);
					}
					break;
				case 11:
					if (button_2_is_pressed()) {
						step_denominator = user_add_witout_overflow(step_denominator, 10);
					} else if (button_3_is_pressed()) {
						step_denominator = user_add_witout_overflow(step_denominator, -10);
					}
					break;
				case 12:
					if (button_2_is_pressed()) {
						step_denominator = user_add_witout_overflow(step_denominator, 1);
					} else if (button_3_is_pressed()) {
						step_denominator = user_add_witout_overflow(step_denominator, -1);
					}
					break;
				default:
					break;
			}	
		}
		
		if (step_denominator == 0) {
			step_denominator = 1;	
		}

		while(button_status())
			;
		_delay_ms(100);
	}
}
