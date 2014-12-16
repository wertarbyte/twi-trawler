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
#include "cmds.h"

#define N_MOTORS 2

#define TWI_ADDRESS (~0xF7)

struct motor_conf_t motor[N_MOTORS];

static void check_bound_switches(uint8_t m_id) {
	switch(m_id) {
		case 0:
			if ((~PIN_SW_A_ZERO & (1<<BIT_SW_A_ZERO))) {
				motor[m_id].pos = 0;
			} else if ((~PIN_SW_A_END & (1<<BIT_SW_A_END))) {
				motor[m_id].pos = 2;
			} else {
				motor[m_id].pos = 1;
			}
			break;
		case 1:
			if ((~PIN_SW_B_ZERO & (1<<BIT_SW_B_ZERO))) {
				motor[m_id].pos = 0;
			} else if ((~PIN_SW_B_END & (1<<BIT_SW_B_END))) {
				motor[m_id].pos = 2;
			} else {
				motor[m_id].pos = 1;
			}
			break;
		default:
			return;

	}
	/* if we have reached the end, stop the motor */
	if ( (motor[m_id].pos == 0 && motor[m_id].dir == MOTOR_DIR_CW) ||
	     (motor[m_id].pos == 2 && motor[m_id].dir == MOTOR_DIR_CCW) ) {
		motor[m_id].dir = MOTOR_DIR_STOPPED;
	}
}

static void check_target_direction(uint8_t m_id) {
	if (motor[m_id].target > motor[m_id].pos) {
		motor[m_id].dir = MOTOR_DIR_CW;
	} else if (motor[m_id].target < motor[m_id].pos) {
		motor[m_id].dir = MOTOR_DIR_CCW;
	} else {
		motor[m_id].dir = MOTOR_DIR_STOPPED;
	}
}

static void set_motor(uint8_t m_id) {
	struct motor_conf_t *mc = &motor[m_id];
	switch(mc->mode) {
		case MOTOR_MODE_FREE:
			motor_set_direction(m_id, mc->dir);
			motor_set_speed(m_id, mc->speed);
			break;
		case MOTOR_MODE_BOUNDED:
			check_target_direction(m_id);
			check_bound_switches(m_id);
			motor_set_direction(m_id, mc->dir);
			motor_set_speed(m_id, (mc->dir != MOTOR_DIR_STOPPED) ? 0xFF : 0x00);
			break;
		default:
			/* failsafe, stop everything */
			motor_set_direction(m_id, MOTOR_DIR_STOPPED);
			motor_set_speed(m_id, 0);
	}
}

uint8_t twiReadCallback(uint8_t addr, uint8_t counter, uint8_t *data) {
	return 1;
}

uint8_t twiWriteCallback(uint8_t addr, uint8_t counter, uint8_t *data) {
	uint8_t m_id = addr & 0x01;
	switch(addr & ~0x01) {
		case CMD_ADDR_MODE:
			switch(*data) {
				case MOTOR_MODE_FREE:
				case MOTOR_MODE_BOUNDED:
					motor[m_id].speed = 0;
					motor[m_id].pos = 1;
					motor[m_id].flags &= ~(MOTOR_FLAG_CALIBRATED);
					motor[m_id].dir = MOTOR_DIR_STOPPED;
					motor[m_id].mode = *data;
					break;
			}
			return 0;
		case CMD_ADDR_SPEED:
			motor[m_id].speed = *data;
			return 0;
		case CMD_ADDR_DIR:
			motor[m_id].dir = *data;
			return 0;
		case CMD_ADDR_GOTO:
			motor[m_id].target = *data;
			return 0;
	}
	return 1;
}

int main(void) {
	memset(motor, 0, sizeof(motor));

	usiTwiSlaveInit(TWI_ADDRESS);
	usiTwiSetReadCallback(&twiReadCallback);
	usiTwiSetWriteCallback(&twiWriteCallback);
	
	/* configure direction outputs */
	DDR_MOTOR_A_DIR_L |= (1<<BIT_MOTOR_A_DIR_L);
	DDR_MOTOR_A_DIR_R |= (1<<BIT_MOTOR_A_DIR_R);
	DDR_MOTOR_B_DIR_L |= (1<<BIT_MOTOR_B_DIR_L);
	DDR_MOTOR_B_DIR_R |= (1<<BIT_MOTOR_B_DIR_R);

	/* configure switch inputs */
	DDR_SW_A_ZERO &= ~(1<<BIT_SW_A_ZERO);
	DDR_SW_A_END  &= ~(1<<BIT_SW_A_END);
	DDR_SW_B_ZERO &= ~(1<<BIT_SW_B_ZERO);
	DDR_SW_B_END  &= ~(1<<BIT_SW_B_END);
	/* enable pull-up resistors */
	PORT_SW_A_ZERO |= (1<<BIT_SW_A_ZERO);
	PORT_SW_A_END  |= (1<<BIT_SW_A_END);
	PORT_SW_B_ZERO |= (1<<BIT_SW_B_ZERO);
	PORT_SW_B_END  |= (1<<BIT_SW_B_END);

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

	while (1) {
		for (uint8_t i=0; i<N_MOTORS; i++) {
			set_motor(i);
		}
	}
	return 0;
}
