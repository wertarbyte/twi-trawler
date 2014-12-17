#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include "usiTwiSlave.h"

#include "motor_ctrl.h"
#include "motor_types.h"
#include "motor.h"
#include "pin_wiring.h"
#include "cmds.h"

#define N_MOTORS 2

#define POS_MAX ((uint32_t)0xFFFFFFFF - 1)
#define POS_UNKNOWN ((uint32_t)0xFFFFFFFF)

#define TWI_BASE_ADDRESS (~0xF7)

struct motor_conf_t motor[N_MOTORS];

static void check_encoder(uint8_t m_id) {
	static uint8_t last[N_MOTORS];
	uint8_t now;
	if (motor[m_id].pos == POS_UNKNOWN) {
		return;
	}
	switch(m_id) {
		case 0:
			now = (~PIN_SW_A_END & (1<<BIT_SW_A_END));
			break;
		case 1:
			now = (~PIN_SW_B_END & (1<<BIT_SW_B_END));
			break;
	}
	// just triggered
	if (now && !last[m_id]) {
		switch(motor[m_id].dir) {
			case MOTOR_DIR_FORWARD:
				motor[m_id].pos++;
				break;
			case MOTOR_DIR_BACK:
				motor[m_id].pos--;
				break;
			case MOTOR_DIR_STOPPED:
				/* strange, but we ignore this for now */
				break;
		}
	}
	last[m_id] = now;
}

static void check_bound_switches(uint8_t m_id, uint8_t end_sw) {
	switch(m_id) {
		case 0:
			if ((~PIN_SW_A_ZERO & (1<<BIT_SW_A_ZERO))) {
				motor[m_id].pos = 0;
			} else if (end_sw) {
				if ((~PIN_SW_A_END & (1<<BIT_SW_A_END))) {
					motor[m_id].pos = POS_MAX;
				} else {
					motor[m_id].pos = 1;
				}
			}
			break;
		case 1:
			if ((~PIN_SW_B_ZERO & (1<<BIT_SW_B_ZERO))) {
				motor[m_id].pos = 0;
			} else if (end_sw) {
				if ((~PIN_SW_B_END & (1<<BIT_SW_B_END))) {
					motor[m_id].pos = POS_MAX;
				} else {
					motor[m_id].pos = 1;
				}
			}
			break;
		default:
			return;

	}
	/* if we have reached the end, stop the motor */
	if ( (motor[m_id].pos ==       0 && motor[m_id].dir == MOTOR_DIR_BACK) ||
	     (motor[m_id].pos == POS_MAX && motor[m_id].dir == MOTOR_DIR_FORWARD) ) {
		motor[m_id].dir = MOTOR_DIR_STOPPED;
	}
	/* Did we finish a calibration sequence? */
	if (motor[m_id].flags & MOTOR_FLAG_CALIBRATING) {
		motor[m_id].flags &= ~(MOTOR_FLAG_CALIBRATING);
		motor[m_id].flags |= MOTOR_FLAG_CALIBRATED;
	}
}

static void check_target_direction(uint8_t m_id) {
	if (motor[m_id].pos == POS_UNKNOWN && !(motor[m_id].flags & MOTOR_FLAG_CALIBRATING)) {
		return;
	}

	if (motor[m_id].target > motor[m_id].pos) {
		motor[m_id].dir = MOTOR_DIR_FORWARD;
	} else if (motor[m_id].target < motor[m_id].pos) {
		motor[m_id].dir = MOTOR_DIR_BACK;
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
		case MOTOR_MODE_ENCODER:
			check_encoder(m_id);
		case MOTOR_MODE_BOUNDED:
			check_target_direction(m_id);
			check_bound_switches(m_id, (mc->mode == MOTOR_MODE_BOUNDED));
			motor_set_direction(m_id, mc->dir);
			motor_set_speed(m_id, (mc->dir != MOTOR_DIR_STOPPED) ? 0xFF : 0x00);
			break;
		default:
			/* failsafe, stop everything */
			motor_set_direction(m_id, MOTOR_DIR_STOPPED);
			motor_set_speed(m_id, 0);
	}
}

static uint8_t getByte(uint8_t *b, size_t s, uint8_t count) {
	if (count < s) {
		return b[count];
	} else {
		return 0;
	}
}

static void setByte(uint8_t *b, size_t s, uint8_t data, uint8_t count) {
	if (count < s) {
		b[count] = data;
	}
}

#define TWI_BUF_SIZE 4
static uint8_t twi_buf[TWI_BUF_SIZE];

uint8_t twiReadCallback(uint8_t addr, uint8_t counter, uint8_t *data) {
	uint8_t m_id = addr & 0x01;
	switch(addr & ~0x01) {
		case CMD_ADDR_MODE:
			*data = motor[m_id].mode;
			return 1;
		case CMD_ADDR_SPEED:
			*data = motor[m_id].speed;
			return 1;
		case CMD_ADDR_DIR:
			*data = motor[m_id].dir;
			return 1;
		case CMD_ADDR_GOTO:
			if (counter == 0) memcpy(&twi_buf, &motor[m_id].target, sizeof(motor[m_id].target));
			*data = getByte(&twi_buf[0], sizeof(motor[m_id].target), counter);
			return 1;
		case CMD_ADDR_POS:
			if (counter == 0) memcpy(&twi_buf, &motor[m_id].pos, sizeof(motor[m_id].pos));
			*data = getByte(&twi_buf[0], sizeof(motor[m_id].pos), counter);
			return 1;
	}
	return 0;
}

uint8_t twiWriteCallback(uint8_t addr, uint8_t counter, uint8_t data) {
	uint8_t m_id = addr & 0x01;
	uint8_t cmd = addr & ~0x01;
	switch(cmd) {
		case CMD_ADDR_MODE:
			switch(data) {
				case MOTOR_MODE_FREE:
				case MOTOR_MODE_BOUNDED:
				case MOTOR_MODE_ENCODER:
					motor[m_id].speed = 0;
					motor[m_id].pos = POS_UNKNOWN;
					motor[m_id].flags = 0;
					motor[m_id].dir = MOTOR_DIR_STOPPED;
					motor[m_id].mode = data;
					break;
			}
			return 1;
		case CMD_ADDR_SPEED:
			motor[m_id].speed = data;
			return 1;
		case CMD_ADDR_DIR:
			motor[m_id].dir = data;
			return 1;
		case CMD_ADDR_GOTO:
			setByte(&twi_buf[0], sizeof(motor[m_id].target), data, counter);
			if (counter == 3) memcpy(&motor[m_id].target, &twi_buf[0], sizeof(motor[m_id].target));
			return 1;
		case CMD_ADDR_CALIB:
			motor[m_id].flags |= MOTOR_FLAG_CALIBRATING;
			motor[m_id].pos = POS_UNKNOWN;
			motor[m_id].target = 0;
			return 1;
	}
	return 0;
}

static uint8_t twiRead(uint8_t reg, uint8_t c) {
	uint8_t d = 0;
	twiReadCallback(reg, c, &d);
	return d;
}

static void twiWrite(uint8_t reg, uint8_t c, uint8_t val) {
	twiWriteCallback(reg, c, val);
}

int main(void) {
	memset(motor, 0, sizeof(motor));

	/* check address pin */
	DDR_TWI_ADDR_BIT_1 &= ~(1<<BIT_TWI_ADDR_BIT_1);
	PORT_TWI_ADDR_BIT_1 |= (1<<BIT_TWI_ADDR_BIT_1);
	_delay_ms(5);
	/* if PD6 is connected to GND, set the lowest bit of the TWI address */
	uint8_t twi_bit = (~PIN_TWI_ADDR_BIT_1 & (1<<BIT_TWI_ADDR_BIT_1)) ? 0x01 : 0x00;
	PORT_TWI_ADDR_BIT_1 &= ~(1<<BIT_TWI_ADDR_BIT_1);

	usiTwiSlaveInit(TWI_BASE_ADDRESS | twi_bit, &twiRead, &twiWrite);

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

