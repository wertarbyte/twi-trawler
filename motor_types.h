#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H 1

enum motor_mode_t {
	MOTOR_MODE_UNDEF = 0,
	MOTOR_MODE_FREE,
	MOTOR_MODE_BOUNDED,
	MOTOR_MODE_ENCODER,
};

enum motor_dir_t {
	MOTOR_DIR_STOPPED,
	MOTOR_DIR_FORWARD,
	MOTOR_DIR_BACK,
};

typedef uint32_t motor_pos_t;


#define MOTOR_FLAG_CALIBRATED  (1<<0)
#define MOTOR_FLAG_CALIBRATING (1<<1)

struct motor_conf_t {
	enum motor_mode_t mode;
	enum motor_dir_t dir;
	uint8_t flags;
	motor_pos_t pos;
	motor_pos_t target;
	uint8_t speed;
};

#endif /* MOTOR_TYPES_H */
