#ifndef MICROB0T_TYPES_H
#define MICROB0T_TYPES_H
#include <stdbool.h>
#include <stdint.h>

typedef enum { MB_LEFT=0, MB_RIGHT=1 } mb_wheel_t;
typedef enum { MB_DIR_REVERSE=-1, MB_DIR_STOP=0, MB_DIR_FORWARD=1 } mb_direction_t;
typedef enum { MB_SPEED_SLOW=0, MB_SPEED_MEDIUM=1, MB_SPEED_FAST=2 } mb_speed_t;
typedef enum {
    MB_STATE_INIT, MB_STATE_WAIT_START, MB_STATE_FORWARD, MB_STATE_STOPPING,
    MB_STATE_TURN_RIGHT, MB_STATE_TURN_LEFT, MB_STATE_REVERSE,
    MB_STATE_RECHECK, MB_STATE_SAFE_STOP, MB_STATE_SENSOR_FAULT
} mb_state_t;
typedef enum { MB_PATH_CLEAR, MB_PATH_BLOCKED, MB_PATH_INVALID } mb_path_t;
typedef enum { MB_SCENARIO_BASIC=1, MB_SCENARIO_TURN_SELECT=2, MB_SCENARIO_RECOVERY=3 } mb_scenario_t;
typedef struct {
    uint16_t distance_cm;
    bool valid;
    bool timeout;
} mb_distance_t;
typedef struct {
    mb_state_t state;
    mb_speed_t selected_speed;
    uint8_t left_speed_percent, right_speed_percent;
    uint16_t last_distance_cm;
    uint8_t recovery_cycle;
    bool started, sensor_fault, stop_latched;
} mb_status_t;
#endif
