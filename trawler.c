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

#define POS_MIN ((motor_pos_t)0)
#define POS_MAX ((motor_pos_t)0xFFFF - 1)
#define POS_UNKNOWN ((motor_pos_t)0xFFFF)
#define POS_TRANSIT (POS_MAX/2)

#define TWI_BASE_ADDRESS (~0xF7)

struct motor_conf_t motor[N_MOTORS];

static volatile uint8_t enc_pulses[N_MOTORS];

ISR (INT0_vect) {
	enc_pulses[0]++;
}

ISR (INT1_vect) {
	enc_pulses[1]++;
}

static void check_encoder(uint8_t m_id) {
	uint8_t count = enc_pulses[m_id];
	enc_pulses[m_id] -= count;
	motor[m_id].odometer += count;
	if (motor[m_id].pos != POS_UNKNOWN) {
		switch(motor[m_id].enc_dir) {
			case MOTOR_DIR_FORWARD:
				motor[m_id].pos += count;
				break;
			case MOTOR_DIR_BACK:
				if (count < motor[m_id].pos) {
					motor[m_id].pos -= count;
				} else {
					motor[m_id].pos = 0;
				}
				break;
			case MOTOR_DIR_STOPPED:
				/* strange, but we ignore this for now */
				break;
		}
	}
}

static void check_bound_switches(uint8_t m_id, uint8_t end_sw) {
	switch(m_id) {
		case 0:
			if ((~PIN_SW_A_ZERO & (1<<BIT_SW_A_ZERO))) {
				motor[m_id].pos = POS_MIN;
			} else if (end_sw) {
				if ((~PIN_SW_A_END & (1<<BIT_SW_A_END))) {
					motor[m_id].pos = POS_MAX;
				} else {
					motor[m_id].pos = POS_TRANSIT;
				}
			}
			break;
		case 1:
			if ((~PIN_SW_B_ZERO & (1<<BIT_SW_B_ZERO))) {
				motor[m_id].pos = POS_MIN;
			} else if (end_sw) {
				if ((~PIN_SW_B_END & (1<<BIT_SW_B_END))) {
					motor[m_id].pos = POS_MAX;
				} else {
					motor[m_id].pos = POS_TRANSIT;
				}
			}
			break;
		default:
			return;

	}
	/* if we have reached the end, stop the motor */
	if ( (motor[m_id].pos == POS_MIN && motor[m_id].dir == MOTOR_DIR_BACK) ||
	     (motor[m_id].pos == POS_MAX && motor[m_id].dir == MOTOR_DIR_FORWARD) ) {
		motor[m_id].dir = MOTOR_DIR_STOPPED;
		motor[m_id].enc_dir = motor[m_id].dir;
	}

	/* Did we finish a calibration sequence? */
	if ((motor[m_id].pos == POS_MIN) && (motor[m_id].flags & MOTOR_FLAG_CALIBRATING)) {
		motor[m_id].flags &= ~(MOTOR_FLAG_CALIBRATING);
		motor[m_id].flags |= MOTOR_FLAG_CALIBRATED;
		motor[m_id].odometer = 0;
	}
}

static motor_pos_t distance_to_target(uint8_t m_id) {
	if (motor[m_id].pos >= motor[m_id].target) {
		return motor[m_id].pos - motor[m_id].target;
	} else {
		return motor[m_id].target - motor[m_id].pos;
	}
}

static uint8_t approach_speed(uint8_t m_id) {
	motor_pos_t d = distance_to_target(m_id);
	if (d <= 150) {
		return 90;
	}
	if (d <= 256) {
		return 128;
	}
	return 0xFF;
}

#define STOP_COUNT_MAX 255
static void check_target_direction(uint8_t m_id) {
	static uint8_t stop_counter[N_MOTORS];

	if (motor[m_id].pos == POS_UNKNOWN && !(motor[m_id].flags & MOTOR_FLAG_CALIBRATING)) {
		return;
	}

	enum motor_dir_t target_dir = MOTOR_DIR_STOPPED;
	if (motor[m_id].target > motor[m_id].pos) {
		target_dir = MOTOR_DIR_FORWARD;
	} else {
		target_dir = MOTOR_DIR_BACK;
	}

	/* do we need to change direction? */
	if (target_dir != motor[m_id].dir) {
		/* is the motor currently stopped? */
		if ((motor[m_id].dir == MOTOR_DIR_STOPPED) && (stop_counter[m_id] == STOP_COUNT_MAX)) {
			/* start it in the new direction */
			motor[m_id].dir = target_dir;
			/* also use the encoder in the new direction */
			motor[m_id].enc_dir = target_dir;
		} else {
			if (motor[m_id].dir != MOTOR_DIR_STOPPED) {
				stop_counter[m_id] = 0;
			} else {
				stop_counter[m_id]++;
			}
			/* stop the motor, but keep counting in the old direction */
			motor[m_id].dir = MOTOR_DIR_STOPPED;
		}
	}
}

#define STAB_COUNT_MAX 1024

static uint8_t target_reached(uint8_t m_id) {
	return (motor[m_id].target == motor[m_id].pos) && (motor[m_id].stab_count == STAB_COUNT_MAX);
}

static void check_pos_stability(uint8_t m_id) {
	if (motor[m_id].target == motor[m_id].pos) {
		if (motor[m_id].stab_count != STAB_COUNT_MAX) {
			motor[m_id].stab_count++;
		}
	} else {
			motor[m_id].stab_count = 0;
	}
}

static void set_motor(uint8_t m_id) {
	struct motor_conf_t *mc = &motor[m_id];
	switch(mc->mode) {
		case MOTOR_MODE_FREE:
			motor_set_speed(m_id, mc->speed);
			motor_set_direction(m_id, mc->dir);
			break;
		case MOTOR_MODE_ENCODER:
			check_encoder(m_id);
		case MOTOR_MODE_BOUNDED:
			check_target_direction(m_id);
			check_bound_switches(m_id, (mc->mode == MOTOR_MODE_BOUNDED));
			motor_set_speed(m_id, approach_speed(m_id));
			motor_set_direction(m_id, mc->dir);
			check_pos_stability(m_id);
			break;
		default:
			/* failsafe, stop everything */
			motor_set_speed(m_id, 0);
			motor_set_direction(m_id, MOTOR_DIR_STOPPED);
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

uint8_t twiReadCallback(uint8_t addr, uint8_t counter) {
	uint8_t m_id = addr & 0x01;
	switch(addr & ~0x01) {
		case CMD_ADDR_MODE:
			return motor[m_id].mode;
		case CMD_ADDR_SPEED:
			return motor[m_id].speed;
		case CMD_ADDR_DIR:
			return motor[m_id].dir;
		case CMD_ADDR_GOTO:
			if (counter == 0) memcpy(&twi_buf, &motor[m_id].target, sizeof(motor[m_id].target));
			return getByte(&twi_buf[0], sizeof(motor[m_id].target), counter);
		case CMD_ADDR_POS:
			if (counter == 0) memcpy(&twi_buf, &motor[m_id].pos, sizeof(motor[m_id].pos));
			return getByte(&twi_buf[0], sizeof(motor[m_id].pos), counter);
		case CMD_ADDR_ODO:
			if (counter == 0) memcpy(&twi_buf, &motor[m_id].odometer, sizeof(motor[m_id].odometer));
			return getByte(&twi_buf[0], sizeof(motor[m_id].odometer), counter);
		case CMD_ADDR_TARGET_REACHED:
			return target_reached(m_id);
	}
	return 0;
}

void twiWriteCallback(uint8_t addr, uint8_t counter, uint8_t data) {
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
					motor[m_id].enc_dir = MOTOR_DIR_STOPPED;
					motor[m_id].dir = MOTOR_DIR_STOPPED;
					motor[m_id].mode = data;
					break;
			}
			break;
		case CMD_ADDR_SPEED:
			motor[m_id].speed = data;
			break;
		case CMD_ADDR_DIR:
			motor[m_id].dir = data;
			break;
		case CMD_ADDR_GOTO:
			setByte(&twi_buf[0], sizeof(motor[m_id].target), data, counter);
			if (counter == sizeof(motor[m_id].target)-1) memcpy(&motor[m_id].target, &twi_buf[0], sizeof(motor[m_id].target));
			break;
		case CMD_ADDR_CALIB:
			motor[m_id].flags |= MOTOR_FLAG_CALIBRATING;
			motor[m_id].pos = POS_UNKNOWN;
			motor[m_id].target = POS_MIN;
			break;
	}
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

	usiTwiSlaveInit(TWI_BASE_ADDRESS | twi_bit, &twiReadCallback, &twiWriteCallback);

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

	TCCR0B = (1 << CS00) | (0 << CS01) | (1 << CS02);

	/* prepare interrupts for encoder input */

	/* interrupt on falling edge */
	MCUCR |= (1<<ISC01) | (0<<ISC00);
	MCUCR |= (1<<ISC11) | (0<<ISC10);
	/* enable INT0 and INT1 */
	GIMSK  |= (1<<INT0 | 1<<INT1);

	sei();

	while (1) {
		for (uint8_t i=0; i<N_MOTORS; i++) {
			set_motor(i);
		}
	}
	return 0;
}

