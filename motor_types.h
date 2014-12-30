#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H 1

#include "motor_ctrl.h"

typedef uint16_t motor_pos_t;

typedef uint32_t time_t;

#define MOTOR_FLAG_CALIBRATED  (1<<0)
#define MOTOR_FLAG_CALIBRATING (1<<1)

struct motor_conf_t {
	enum motor_mode_t mode;
	enum motor_dir_t dir;
	enum motor_dir_t enc_dir;
	uint8_t flags;
	motor_pos_t pos;
	motor_pos_t target;
	time_t last_movement;
	uint8_t speed;
	uint16_t odometer;
	volatile uint8_t enc_pulses;
};

#endif /* MOTOR_TYPES_H */
