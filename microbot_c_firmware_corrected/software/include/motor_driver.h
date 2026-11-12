#ifndef MICROB0T_MOTOR_DRIVER_H
#define MICROB0T_MOTOR_DRIVER_H
#include <stdint.h>
#include "microbot_types.h"
void motor_init(void);
void motor_stop(void);
void motor_set_wheels(mb_direction_t left_dir, uint8_t left_speed,
                      mb_direction_t right_dir, uint8_t right_speed);
void motor_forward(uint8_t left_speed, uint8_t right_speed);
void motor_reverse(uint8_t left_speed, uint8_t right_speed);
void motor_turn_right(uint8_t speed);
void motor_turn_left(uint8_t speed);
#endif
