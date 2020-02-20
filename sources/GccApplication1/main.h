#ifndef MAIN_H_
#define MAIN_H_

#define LCD_DISPLAY_ADDRESS 0x27

#define SUPPORT_RECALCULATION_SPEED 128 // 16 MHz / 64 / 128 / 2 ~ 1 kHz   deleno 2 protoze v jednom kroku nastavime puls na Driveru na 1 a pak v druhem na 0

#define STEPS_FOR_ONE_TURN 600u

uint16_t get_current_spindle_angle_overflow();

#endif /* MAIN_H_ */