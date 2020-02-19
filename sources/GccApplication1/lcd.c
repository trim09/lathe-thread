#include "lcd.h"

#include "i2cmaster.h"

#include <stdarg.h>
#include <stdio.h>
#include <util/delay.h>


static uint8_t lcd_displayparams;
static char lcd_buffer[LCD_COL_COUNT + 1];

#define BACKLIGHT_ON (1 << LCD_BACKLIGHT)
#define RS_ON (1 << LCD_RS)
#define RS_OFF 0x00
#define EN_ON (1 << LCD_EN)
#define EN_OFF 0x00

void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
	i2c_write(rs | EN_OFF | BACKLIGHT_ON | (nibble << 4));
	i2c_write(rs | EN_ON | BACKLIGHT_ON | (nibble << 4));
	i2c_write(rs | EN_OFF | BACKLIGHT_ON | (nibble << 4));
	_delay_ms(0.3);	// If delay less than this value, the data is not correctly displayed
}

void lcd_send(uint8_t value, uint8_t rs) {
	lcd_write_nibble(value >> 4, rs);
	lcd_write_nibble(value, rs);
}

void lcd_command(uint8_t command) {
  lcd_send(command, RS_OFF);
}

void lcd_write(uint8_t value) {
  lcd_send(value, RS_ON);
}


void lcd_on(void) {
	lcd_displayparams |= LCD_DISPLAYON;
	lcd_command(LCD_DISPLAYCONTROL | lcd_displayparams);
}

void lcd_off(void) {
	lcd_displayparams &= ~LCD_DISPLAYON;
	lcd_command(LCD_DISPLAYCONTROL | lcd_displayparams);
}

void lcd_clear(void) {
	lcd_command(LCD_CLEARDISPLAY);
	_delay_ms(2);
}

void lcd_return_home(void) {
	lcd_command(LCD_RETURNHOME);
	_delay_ms(2);
}

void lcd_enable_blinking(void) {
	lcd_displayparams |= LCD_BLINKON;
	lcd_command(LCD_DISPLAYCONTROL | lcd_displayparams);
}

void lcd_disable_blinking(void) {
	lcd_displayparams &= ~LCD_BLINKON;
	lcd_command(LCD_DISPLAYCONTROL | lcd_displayparams);
}

void lcd_enable_cursor(void) {
	lcd_displayparams |= LCD_CURSORON;
	lcd_command(LCD_DISPLAYCONTROL | lcd_displayparams);
}

void lcd_disable_cursor(void) {
	lcd_displayparams &= ~LCD_CURSORON;
	lcd_command(LCD_DISPLAYCONTROL | lcd_displayparams);
}

void lcd_scroll_left(void) {
	lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void lcd_scroll_right(void) {
	lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void lcd_set_left_to_right(void) {
	lcd_displayparams |= LCD_ENTRYLEFT;
	lcd_command(LCD_ENTRYMODESET | lcd_displayparams);
}

void lcd_set_right_to_left(void) {
	lcd_displayparams &= ~LCD_ENTRYLEFT;
	lcd_command(LCD_ENTRYMODESET | lcd_displayparams);
}

void lcd_enable_autoscroll(void) {
	lcd_displayparams |= LCD_ENTRYSHIFTINCREMENT;
	lcd_command(LCD_ENTRYMODESET | lcd_displayparams);
}

void lcd_disable_autoscroll(void) {
	lcd_displayparams &= ~LCD_ENTRYSHIFTINCREMENT;
	lcd_command(LCD_ENTRYMODESET | lcd_displayparams);
}

void lcd_create_char(uint8_t location, uint8_t *charmap) {
	lcd_command(LCD_SETCGRAMADDR | ((location & 0x7) << 3));
	for (int i = 0; i < 8; i++) {
		lcd_write(charmap[i]);
	}
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
	static uint8_t offsets[] = { 0x00, 0x40, 0x14, 0x54 };

	if (row >= LCD_ROW_COUNT) {
		row = LCD_ROW_COUNT - 1;
	}

	lcd_command(LCD_SETDDRAMADDR | (col + offsets[row]));
}

void lcd_puts(char *string) {
	for (char *it = string; *it; it++) {
		lcd_write(*it);
	}
}

void lcd_printf(char *format, ...) {
	va_list args;

	va_start(args, format);
	vsnprintf(lcd_buffer, LCD_COL_COUNT + 1, format, args);
	va_end(args);

	lcd_puts(lcd_buffer);
}


void lcd_init(void) {

  // Wait for LCD to become ready (docs say 15ms+)
  _delay_ms(15);

  //i2c_write(0x00); //TODO
						
  //_delay_ms(4.1);

  lcd_write_nibble(0x03, RS_OFF); // Switch to 4 bit mode
  _delay_ms(4.1);

  lcd_write_nibble(0x03, RS_OFF); // 2nd time
  _delay_ms(4.1);

  lcd_write_nibble(0x03, RS_OFF); // 3rd time
  _delay_ms(4.1);

  lcd_write_nibble(0x02, RS_OFF); // Set 8-bit mode (?)

  lcd_command(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);

  lcd_displayparams = LCD_CURSOROFF | LCD_BLINKOFF;
  lcd_command(LCD_DISPLAYCONTROL | lcd_displayparams);
  lcd_clear();
  lcd_set_left_to_right();
  lcd_return_home();
  lcd_on();
}

