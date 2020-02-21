#ifndef SUPPORT_H_
#define SUPPORT_H_

#include <stdint.h>

void support_init();

//void support_set_fraction(float fraction);
void support_set_fraction(uint8_t numerator, uint8_t denominator);

void support_spindle_incremented_event();
void support_spindle_decremented_event();

uint32_t get_actual_support_position();
uint32_t get_required_support_position();

#endif /* SUPPORT_H_ */