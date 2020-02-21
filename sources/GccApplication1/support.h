#ifndef SUPPORT_H_
#define SUPPORT_H_

#include <stdint.h>

void support_init();
void support_set_fraction(float fraction);

void recalculate_support_position(uint32_t current_spindle_revolution_steps);

uint32_t get_actual_support_position();
uint32_t get_required_support_position();

#endif /* SUPPORT_H_ */