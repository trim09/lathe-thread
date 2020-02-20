#ifndef BUTTONS_H_
#define BUTTONS_H_

#include <stdint.h>

void init_buttons();
uint8_t button_1_is_pressed();
uint8_t button_2_is_pressed();
uint8_t button_3_is_pressed();
uint8_t button_4_is_pressed();
uint8_t button_5_is_pressed();
uint8_t button_status();

#endif /* BUTTONS_H_ */