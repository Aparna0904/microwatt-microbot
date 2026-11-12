#ifndef MICROB0T_CONFIG_H
#define MICROB0T_CONFIG_H
#include <stdint.h>

/* User-selected operating parameters. Values remain tunable after calibration. */
#define MB_OBSTACLE_THRESHOLD_CM       20u
#define MB_SENSOR_INTERVAL_MS          100u
#define MB_SENSOR_RETRIES              3u
#define MB_RIGHT_TURN_MS               500u
#define MB_LEFT_TURN_MS                500u
#define MB_REVERSE_MAX_MS              300u
#define MB_MAX_RECOVERY_CYCLES         2u
#define MB_SPEED_RAMP_STEP_MS          50u
#define MB_SPEED_RAMP_STEP_PERCENT     2u
#define MB_SENSOR_CONVERSION_TIMEOUT_MS 40u
#define MB_UART_LINE_MAX               48u

/* Initial software percentages only. Calibrate pulse widths and safe speeds physically. */
#define MB_SPEED_SLOW_DEFAULT          25u
#define MB_SPEED_MEDIUM_DEFAULT        45u
#define MB_SPEED_FAST_DEFAULT          65u
#define MB_TURN_SPEED_DEFAULT          35u
#define MB_REVERSE_SPEED_DEFAULT      30u

/* Conservative initial pulse ranges; validate against the exact servo and supply. */
#define MB_SERVO_PERIOD_US             20000u
#define MB_SERVO_MIN_PULSE_US           700u
#define MB_SERVO_MAX_PULSE_US          2300u
#define MB_SERVO_NEUTRAL_US            1500u

#define MB_UART_DEFAULT_DETAILED       1u
#define MB_STOP_LATCHED_DEFAULT        1u

#endif
