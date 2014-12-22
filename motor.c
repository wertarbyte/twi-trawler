#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include "motor_types.h"
#include "motor.h"
#include "pin_wiring.h"

void motor_set_speed(uint8_t motor_id, uint8_t speed) {
	switch(motor_id) {
		case 0:
			OCR0A  = speed;
			break;
		case 1:
			OCR0B  = speed;
			break;
	}
}

void motor_set_direction(uint8_t motor_id, enum motor_dir_t dir) {
	switch(motor_id) {
		case 0:
			switch(dir) {
				case MOTOR_DIR_BACK:
					PORT_MOTOR_A_DIR_L |=  (1<<BIT_MOTOR_A_DIR_L);
					PORT_MOTOR_A_DIR_R &= ~(1<<BIT_MOTOR_A_DIR_R);
					break;
				case MOTOR_DIR_FORWARD:
					PORT_MOTOR_A_DIR_L &= ~(1<<BIT_MOTOR_A_DIR_L);
					PORT_MOTOR_A_DIR_R |=  (1<<BIT_MOTOR_A_DIR_R);
					break;
				case MOTOR_DIR_STOPPED:
				default:
					PORT_MOTOR_A_DIR_L &= ~(1<<BIT_MOTOR_A_DIR_L);
					PORT_MOTOR_A_DIR_R &= ~(1<<BIT_MOTOR_A_DIR_R);
					break;
			}
			break;
		case 1:
			switch(dir) {
				case MOTOR_DIR_BACK:
					PORT_MOTOR_B_DIR_L |=  (1<<BIT_MOTOR_B_DIR_L);
					PORT_MOTOR_B_DIR_R &= ~(1<<BIT_MOTOR_B_DIR_R);
					break;
				case MOTOR_DIR_FORWARD:
					PORT_MOTOR_B_DIR_L &= ~(1<<BIT_MOTOR_B_DIR_L);
					PORT_MOTOR_B_DIR_R |=  (1<<BIT_MOTOR_B_DIR_R);
					break;
				case MOTOR_DIR_STOPPED:
				default:
					PORT_MOTOR_B_DIR_L &= ~(1<<BIT_MOTOR_B_DIR_L);
					PORT_MOTOR_B_DIR_R &= ~(1<<BIT_MOTOR_B_DIR_R);
					break;
			}
			break;
	}
}
