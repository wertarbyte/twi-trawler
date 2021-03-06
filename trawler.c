#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <util/atomic.h>

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

#define DEFAULT_POS_TOLERANCE 5

#define TWI_BASE_ADDRESS (~0xF7)

static struct motor_conf_t motor[N_MOTORS];

static volatile time_t cs_time;

ISR (INT0_vect) {
	motor[0].enc_pulses++;
}

ISR (INT1_vect) {
	motor[1].enc_pulses++;
}

ISR (TIMER1_COMPA_vect) {
	cs_time++;
}

static time_t now(void) {
	time_t r;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		r = cs_time;
	}
	return r;
}

static void check_sensors(uint8_t m_id) {
	struct motor_conf_t *mc = &motor[m_id];
	motor_pos_t pos_before = mc->pos;
	/* check encoder signals */
	uint8_t count = 0;
	if (mc->mode == MOTOR_MODE_ENCODER) {
		count = mc->enc_pulses;
		mc->enc_pulses -= count;
		mc->odometer += count;
		if (mc->pos != POS_UNKNOWN) {
			switch(mc->enc_dir) {
				case MOTOR_DIR_FORWARD:
					mc->pos += count;
					break;
				case MOTOR_DIR_BACK:
					if (count < mc->pos) {
						mc->pos -= count;
					} else {
						mc->pos = 0;
					}
					break;
				case MOTOR_DIR_STOPPED:
					/* strange, but we ignore this for now */
					break;
			}
		}
	}

	/* check bound switches */
	switch(m_id) {
		case 0:
			if ((~PIN_SW_A_ZERO & (1<<BIT_SW_A_ZERO))) {
				mc->pos = POS_MIN;
			} else if (mc->mode == MOTOR_MODE_BOUNDED) {
				if ((~PIN_SW_A_END & (1<<BIT_SW_A_END))) {
					mc->pos = POS_MAX;
				} else {
					mc->pos = POS_TRANSIT;
				}
			}
			break;
		case 1:
			if ((~PIN_SW_B_ZERO & (1<<BIT_SW_B_ZERO))) {
				mc->pos = POS_MIN;
			} else if (mc->mode == MOTOR_MODE_BOUNDED) {
				if ((~PIN_SW_B_END & (1<<BIT_SW_B_END))) {
					mc->pos = POS_MAX;
				} else {
					mc->pos = POS_TRANSIT;
				}
			}
			break;
	}
	/* if we have reached the end, stop the motor */
	if ( (mc->pos == POS_MIN && mc->dir == MOTOR_DIR_BACK) ||
	     (mc->pos == POS_MAX && mc->dir == MOTOR_DIR_FORWARD) ) {
		mc->dir = MOTOR_DIR_STOPPED;
		mc->enc_dir = mc->dir;
	}

	/* Did we finish a calibration sequence? */
	if ((mc->pos == POS_MIN) && (mc->flags & MOTOR_FLAG_CALIBRATING)) {
		mc->flags &= ~(MOTOR_FLAG_CALIBRATING);
		mc->flags |= MOTOR_FLAG_CALIBRATED;
		mc->odometer = 0;
	}

	/* was there movement at all? */
	if (pos_before != mc->pos || count) {
		mc->last_movement = now();
	}
}

static motor_pos_t distance_to_target(uint8_t m_id) {
	struct motor_conf_t *mc = &motor[m_id];
	if (mc->pos >= mc->target) {
		return mc->pos - mc->target;
	} else {
		return mc->target - mc->pos;
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

static uint8_t target_approached(uint8_t m_id) {
	return (distance_to_target(m_id) < motor[m_id].pos_tolerance);
}

static void check_target_direction(uint8_t m_id) {
	struct motor_conf_t *mc = &motor[m_id];

	if (mc->pos == POS_UNKNOWN && !(mc->flags & MOTOR_FLAG_CALIBRATING)) {
		return;
	}

	enum motor_dir_t target_dir = MOTOR_DIR_STOPPED;
	if (! target_approached(m_id)) {
		if (mc->target > mc->pos) {
			target_dir = MOTOR_DIR_FORWARD;
		} else {
			target_dir = MOTOR_DIR_BACK;
		}
	}

	/* do we need to change direction? */
	if (target_dir != mc->dir) {
		/* is the motor currently stopped? Did it rest for a grace period? */
		if ((mc->dir == MOTOR_DIR_STOPPED) && (now() > mc->last_movement + 2)) {
			/* start it in the new direction */
			mc->dir = target_dir;
			/* also use the encoder in the new direction */
			mc->enc_dir = target_dir;
		} else {
			/* stop the motor, but keep counting in the old direction */
			mc->dir = MOTOR_DIR_STOPPED;
		}
	}
}

static uint8_t target_reached(uint8_t m_id) {
	struct motor_conf_t *mc = &motor[m_id];
	return target_approached(m_id) && (mc->dir == MOTOR_DIR_STOPPED) && (now() > mc->last_movement + 3);
}

static void set_motor(uint8_t m_id) {
	struct motor_conf_t *mc = &motor[m_id];
	switch(mc->mode) {
		case MOTOR_MODE_FREE:
			motor_set_speed(m_id, mc->speed);
			motor_set_direction(m_id, mc->dir);
			break;
		case MOTOR_MODE_ENCODER:
		case MOTOR_MODE_BOUNDED:
			check_target_direction(m_id);
			check_sensors(m_id);
			motor_set_speed(m_id, approach_speed(m_id));
			motor_set_direction(m_id, mc->dir);
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
	struct motor_conf_t *mc = &motor[m_id];
	switch(addr & ~0x01) {
		case CMD_ADDR_MODE:
			return mc->mode;
		case CMD_ADDR_SPEED:
			return mc->speed;
		case CMD_ADDR_DIR:
			return mc->dir;
		case CMD_ADDR_GOTO:
			if (counter == 0) memcpy(&twi_buf, &mc->target, sizeof(mc->target));
			return getByte(&twi_buf[0], sizeof(mc->target), counter);
		case CMD_ADDR_POS:
			if (counter == 0) memcpy(&twi_buf, &mc->pos, sizeof(mc->pos));
			return getByte(&twi_buf[0], sizeof(mc->pos), counter);
		case CMD_ADDR_ODO:
			if (counter == 0) memcpy(&twi_buf, &mc->odometer, sizeof(mc->odometer));
			return getByte(&twi_buf[0], sizeof(mc->odometer), counter);
		case CMD_ADDR_TARGET_REACHED:
			return target_reached(m_id);
		case CMD_ADDR_POS_TOLERANCE:
			return mc->pos_tolerance;
	}
	return 0;
}

void twiWriteCallback(uint8_t addr, uint8_t counter, uint8_t data) {
	uint8_t m_id = addr & 0x01;
	uint8_t cmd = addr & ~0x01;
	struct motor_conf_t *mc = &motor[m_id];
	switch(cmd) {
		case CMD_ADDR_MODE:
			switch(data) {
				case MOTOR_MODE_FREE:
				case MOTOR_MODE_BOUNDED:
				case MOTOR_MODE_ENCODER:
					mc->speed = 0;
					mc->pos = POS_UNKNOWN;
					mc->pos_tolerance = DEFAULT_POS_TOLERANCE;
					mc->flags = 0;
					mc->enc_dir = MOTOR_DIR_STOPPED;
					mc->dir = MOTOR_DIR_STOPPED;
					mc->mode = data;
					break;
			}
			break;
		case CMD_ADDR_SPEED:
			mc->speed = data;
			break;
		case CMD_ADDR_DIR:
			mc->dir = data;
			break;
		case CMD_ADDR_GOTO:
			setByte(&twi_buf[0], sizeof(mc->target), data, counter);
			if (counter == sizeof(mc->target)-1) memcpy(&mc->target, &twi_buf[0], sizeof(mc->target));
			break;
		case CMD_ADDR_CALIB:
			mc->flags |= MOTOR_FLAG_CALIBRATING;
			mc->pos = POS_UNKNOWN;
			mc->target = POS_MIN;
			break;
		case CMD_ADDR_POS_TOLERANCE:
			mc->pos_tolerance = data;
			break;
	}
}

int main(void) {
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

	/* configure timer 1 compare interrupt for 0.01 second intervals (cs) */
	TCCR1B = (0 << CS10) | (1 << CS11) | (0 << CS12);
	OCR1A = 0x2710;
	TIMSK = (1<<OCIE1A);

	/* prepare interrupts for encoder input */

	/* interrupt on falling edge */
	MCUCR |= (1<<ISC01) | (0<<ISC00) | (1<<ISC11) | (0<<ISC10);
	/* enable INT0 and INT1 */
	GIMSK  |= (1<<INT0 | 1<<INT1);

	sei();

	uint8_t i = 0;
	while (1) {
		set_motor(i);
		if (++i==N_MOTORS) i=0;
	}
	return 0;
}

