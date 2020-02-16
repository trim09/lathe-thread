/*
 * Created: 1/18/2020 11:14:42 PM
 * Motor ma 200 kroku. Puls 350+350us jen tak tak nestiha (kdyz by se predroztocil, tak by asi stihal, ale jinak ne)
 * Kdyz jsem ho roztacel pozvolna, tak se roztocil az na 25 ot/s (1500 ot/min)
 *
 * Driver provede operaci, kdyz Pulse na nabezne hrane (0 -> 1)
 *
 * Timer 0 - 
 * Timer 1 - prepocitavani pozice supportu
 * Timer 2 - 1x za 1ms - pro pocitani otacek za minutu
 * I2C - Display Hitachi HD44780 na adrese 0x27 (39)
 * PortB.5 = ledka primo na desce
 * PortC.2 = Driver Enable
 * PortC.1 = Driver Pulse
 * PortC.0 = Driver Direction
 * PortD.2 = zeleny kabel od snimace otacek
 * PortD.3 = zluty kabel od snimace otacek
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

//#define SUPPORT_RECALCULATION_SPEED 0x80 // 16 MHz / 64 / 128 / 2 ~ 1 kHz   deleno 2 protoze v jednom kroku nastavime puls na Driveru na 1 a pak v druhem na 0
#define SUPPORT_RECALCULATION_SPEED 200

#define STEPS_FOR_ONE_TURN 600u

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

/******* Angle and position ******/
/* Phase 1 wire -> INT0 (PORTD PIN2) 
   Phase 2 wire -> PORTD PIN1 */
static volatile bool current_direction = true;
static volatile uint16_t current_spindle_angle; // natoceni vretena
static volatile uint16_t current_spindle_angle_overflow; // can overflow
static volatile uint16_t current_spindle_absolute_position;
static volatile uint16_t end_position = UINT16_MAX;

//Rotary Encoder interrupt
ISR(INT0_vect) { //Interrupt Vectors in ATmega328P - page 48
	if (PIND & (1 << PD3)) { // rotating left or right?
		current_direction = true;
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
		current_direction = false;
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
}

static void init_step_counting() {
	// Phase wire 1 - External Interrupts External Interrupts - page 58
	EICRA |= (1 << ISC01) | (1 << ISC00); // INT0 on rising edge
	EIMSK |= (1 << INT0); // Enable INT0
	
	// Phase wire 2 
	DDRD &= ~(1 << PD3); // PORTD PIN1 as input
}

/******* stepper-motor ***************/
static uint32_t stepper_motor_absolute_position = 0;

static void stepper_motor_init() {
	DDRC |= 1 << DDC2; // Driver Enable
	DDRD |= 1 << DDD5; // Driver Pulse
	DDRC |= 1 << DDC0; // Driver Direction
	PORTC |= 1 << PORTC2; // Driver Enable -> 1 = enabled
}

static void stepper_motor_move_step_left() {
	PORTC |= 1 << PORTC0;
	PORTD &= ~(1 << PORTD5);
}

static void stepper_motor_move_step_right() {
	PORTC &= ~(1 << PORTC0);
	PORTD &= ~(1 << PORTD5);
}

static void stepper_motor_finish_move() {
	PORTD |= 1 << PORTD5;
}

static uint8_t stepper_motor_move_was_finished() {
	return PORTD & (1 << PORTD5);
}

static void stepper_motor_move_towards(uint32_t required) {
	if (!stepper_motor_move_was_finished()) {
		stepper_motor_finish_move();
	} else {
		if (stepper_motor_absolute_position < required) {
			stepper_motor_move_step_left();
			stepper_motor_absolute_position++;
		} else if (stepper_motor_absolute_position > required) {
			stepper_motor_move_step_right();
			stepper_motor_absolute_position--;
		}
		/*
		else {// TODO for testing
			stepper_motor_move_step_left();
			stepper_motor_absolute_position++;
		}*/
	}
}


/******* support position *********/
static volatile uint8_t step_multiplier = 123u;
static volatile uint8_t step_divisor = 123u;
static volatile uint32_t requeired_support_position = 0;

static void init_support_position_recalculation() {
	OCR1AH = 0x00; 
	OCR1AL = SUPPORT_RECALCULATION_SPEED;
	TCCR1A = 0x00; // Clear Timer on Compare Match (CTC) Mode
	TCCR1B = (1 << CS10) | (1 << CS11) | (1 << WGM12); // CLK / 64x
	TIMSK1 = 1 << OCIE1A; // enable interrupt  /* TODO bug? */
}

static void recalculate_support_position() {
	uint16_t tmp_current_position;
	uint16_t tmp_current_angle;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		tmp_current_position = current_spindle_absolute_position;
		tmp_current_angle = current_spindle_angle;
	}
		
	if (tmp_current_position == 0) {
		tmp_current_angle = 0;
	} else if (tmp_current_position >= end_position)  {
		tmp_current_angle = 0;
	} else {
		tmp_current_position--; //preskocime pozici 0, protoze tu nepouzivame pro posuv supportu
	}
		
	uint32_t requeired_support_position_tmp = ((((uint32_t)tmp_current_position) * STEPS_FOR_ONE_TURN + tmp_current_angle) * step_multiplier) / step_divisor;
	requeired_support_position = requeired_support_position_tmp;
	stepper_motor_move_towards(requeired_support_position_tmp);
}

ISR(TIMER1_COMPA_vect) { // called as fast as stepper motor can handle 
	recalculate_support_position();
	//PORTD ^= (1 << PORTD5); //for testing
}

/********* rotations per second **************/
static volatile int16_t spindle_revolutions_per_minute = 0;
static uint16_t previous_spindle_revolutions = 0;
static uint16_t one_ms_to_one_second = 0;

static void init_revolution_calculation(void) {
	OCR2A = 125u; // 16 MHz / 128 / 125 = 1 ms
	TCCR2A = 1 << WGM21; // Clear Timer on Compare Match (CTC) Mode
	TCCR2B = (1 << CS22) | (1 << CS20); // CLK / 128x
	TIMSK2 = 1 << OCIE2A; // enable interrupt
}

ISR(TIMER2_COMPA_vect) { // once per 1ms
	if (one_ms_to_one_second++ == 1000u) {
		one_ms_to_one_second = 0; // once per second
		
		uint16_t tmp = current_spindle_angle_overflow;
		
		uint16_t angle_increment_per_second = tmp - previous_spindle_revolutions;
		uint32_t angle_increment_per_minute = 60lu * angle_increment_per_second;
		int16_t spindle_revolutions_per_minute_abs = angle_increment_per_minute / STEPS_FOR_ONE_TURN;
		if (current_direction) {
			spindle_revolutions_per_minute = spindle_revolutions_per_minute_abs;
		} else {
			spindle_revolutions_per_minute = -spindle_revolutions_per_minute_abs;	
		}
		
		previous_spindle_revolutions = tmp;
				
		led_toggle();
	}
}

static void enable_motor_driver() {
	/* TODO */
}


#define COUNTER_TOP_VALUE 10
#define COUNTER_MATCH_VALUE 127
//#define COUNTER_SET_VALUE (COUNTER_MATCH_VALUE - 1)
#define COUNTER_SET_VALUE 126

int main(void) {
	PORTB = 0xFF; /* enable pull up on PORTB */
	PORTC = 0xFF; /* enable pull up on PORTC */
	PORTD = 0xFF; /* enable pull up on PORTD */
		
	led_init();
	stepper_motor_init();
	init_step_counting();
	init_support_position_recalculation();
	init_revolution_calculation();
	i2c_init();
	i2c_start_wait(LCD_DISPLAY_ADDRESS << 1 | I2C_WRITE);	
	lcd_init();
	sei(); // enable interrupts
	
	//while(1) {
		//recalculate_support_position();
	//}
	
	//PORTC |= 1 << PORTC2;
	//PORTC |= 1 << PORTC0;
	
	/*
	OCR0A = COUNTER_TOP_VALUE;
	OCR0B = COUNTER_MATCH_VALUE; // MATCH
	TCCR0A = (1 << COM0B0) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); // fast-PWM, COM0B=1 on timer match, COM0B=0 on timer clear, COM0B to ouutput
	TCCR0B = (1 << CS01) | (1 << CS00) | (1 << WGM02); // 64x prescaller, fast-PWM,
	DDRD |= 1 << DDD5;
	//TIFR0
	
	while(1) {
		
		if (TCNT0 == 1) {
			//_delay_us(1);
			TCNT0 = 56;
		}
	}
	
	
		while(1) {
			_delay_us(500);
			PORTC |= 1 << PORTC1;
			_delay_us(500);
			PORTC &= ~(1 << PORTC1);
			
		}
	*/

		/*
	while(1) {	
				DDRB |= 1 << DDB5;
				while (1) {
					PORTB |= 1 << PORTB5;
					for(uint32_t i = 0; i < 160000 ; i++)
					;
					PORTB &= ~(1 << PORTB5);
					for(uint32_t i = 0; i < 1600000 ; i++)
					;
				}
	}
	*/
	


//i2c_write(1<<LCD_RS);


	
	
	
	
	DDRB |= 1 << DDB5;
    while (1) {

						lcd_set_cursor(0, 0);
						lcd_printf("vreteno:  %-5u %3u", current_spindle_angle, current_spindle_absolute_position);
						lcd_set_cursor(0, 1);
						lcd_printf("            %i ot/min", spindle_revolutions_per_minute);
						lcd_set_cursor(0, 2);
						lcd_printf("support: %-lu", stepper_motor_absolute_position);
						lcd_set_cursor(0, 3);
						lcd_printf("%8i %-lu", (int16_t) (requeired_support_position - stepper_motor_absolute_position), requeired_support_position);

    }
	
		DDRB |= 1 << DDB5;
		while (1) {
			PORTB |= 1 << PORTB5;
			for(uint32_t i = 0; i < 160000 ; i++)
			;
			PORTB &= ~(1 << PORTB5);
			for(uint32_t i = 0; i < 1600000 ; i++)
			;
		}
		
}


