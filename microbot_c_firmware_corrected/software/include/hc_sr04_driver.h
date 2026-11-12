#ifndef MICROB0T_HC_SR04_DRIVER_H
#define MICROB0T_HC_SR04_DRIVER_H
#include <stdbool.h>
#include "microbot_types.h"
void hc_sr04_init(void);
bool hc_sr04_start(void);
bool hc_sr04_poll(mb_distance_t *out);
#endif
