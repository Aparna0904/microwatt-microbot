#ifndef MICROB0T_NAVIGATION_H
#define MICROB0T_NAVIGATION_H
#include "microbot_types.h"
void navigation_init(mb_scenario_t scenario);
void navigation_start(void);
void navigation_stop(void);
void navigation_reset(void);
void navigation_set_speed(mb_speed_t speed);
void navigation_set_wheel_speeds(uint8_t left, uint8_t right);
void navigation_tick(void);
mb_status_t navigation_status(void);
#ifdef MB_HOST_BUILD
#include <stdint.h>
uint32_t navigation_host_next_sensor_deadline(void);
#endif
#endif
