#ifndef MOTOR_H
#define MOTOR_H 1

void motor_set_speed(uint8_t motor_id, uint8_t speed);
void motor_set_direction(uint8_t motor_id, enum motor_dir_t dir);

#endif
