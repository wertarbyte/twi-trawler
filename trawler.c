#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include "usiTwiSlave.h"

#include "motor_types.h"
#include "motor.h"
#include "pin_wiring.h"

#define N_MOTORS 2

#define TWI_ADDRESS (~0xF7)

struct motor_conf_t motor[N_MOTORS];

static void set_motor(uint8_t m_id) {
	struct motor_conf_t *mc = &motor[m_id];
	switch(mc->mode) {
		case MOTOR_MODE_FREE:
			motor_set_direction(m_id, mc->dir);
			motor_set_speed(m_id, mc->speed);
			break;
		case MOTOR_MODE_BOUNDED:
			motor_set_speed(m_id, 0xFF);
			break;
	}
}

int main(void) {
	memset(motor, 0, sizeof(motor));
	_delay_ms(2000);

	usiTwiSlaveInit(TWI_ADDRESS);
	usiTwiSetTransmitWindow( &motor, sizeof(motor) );
	
	/* configure direction outputs */
	DDR_MOTOR_A_DIR_L |= (1<<BIT_MOTOR_A_DIR_L);
	DDR_MOTOR_A_DIR_R |= (1<<BIT_MOTOR_A_DIR_R);
	DDR_MOTOR_B_DIR_L |= (1<<BIT_MOTOR_B_DIR_L);
	DDR_MOTOR_B_DIR_R |= (1<<BIT_MOTOR_B_DIR_R);

	/* configure switch inputs */
	DDR_SW_A_ZERO |= (1<<BIT_SW_A_ZERO);
	DDR_SW_A_END  |= (1<<BIT_SW_A_END);
	DDR_SW_B_ZERO |= (1<<BIT_SW_B_ZERO);
	DDR_SW_B_END  |= (1<<BIT_SW_B_END);

	/* configure timer for PWM */
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
	OCR0A = 0x00;
	OCR0B = 0x00;

	/* PWM output (OC0A) on PB2 */
	DDRB |= (1 << PB2);
	/* PWM output (OC0B) on PD5 */
	DDRD |= (1 << PD5);

	TCCR0B = (1 << CS01);

	sei();
	for (uint8_t i=0; i<2; i++) {
		motor[i].mode = MOTOR_MODE_FREE;
		motor[i].speed = 0x04;
	}

	while (1) {
		for (uint8_t i=0; i<N_MOTORS; i++) {
			set_motor(i);
		}
	}
	return 0;
}

