#ifndef MICROB0T_SERVO_DRIVER_H
#define MICROB0T_SERVO_DRIVER_H
#include <stdint.h>
#include "microbot_types.h"
typedef struct {
    uint16_t neutral_us;
    uint16_t min_us, max_us;
    uint8_t forward_sign; /* +1 or -1: per-servo calibration */
    uint8_t max_speed_percent;
} mb_servo_cal_t;
void servo_init(void);
void servo_set_calibration(mb_wheel_t wheel, const mb_servo_cal_t *cal);
uint16_t servo_pulse_for(mb_wheel_t wheel, mb_direction_t dir, uint8_t speed_percent);
void servo_command(mb_wheel_t wheel, mb_direction_t dir, uint8_t speed_percent);
void servo_stop_all(void);
#endif
