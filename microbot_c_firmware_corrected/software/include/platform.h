#ifndef MICROB0T_PLATFORM_H
#define MICROB0T_PLATFORM_H
#include <stdbool.h>
#include <stdint.h>
#include "microbot_types.h"

/* Hardware-independent HAL. All waits in navigation are non-blocking. */
void platform_init(void);
uint32_t platform_millis(void);
void platform_idle(void);
bool platform_sensor_start(void);
bool platform_sensor_result(mb_distance_t *out);
void platform_servo_set_pulse(mb_wheel_t wheel, uint16_t pulse_us);
bool platform_uart_read_byte(uint8_t *out);
void platform_uart_write(const char *text);

/* Host-test controls. Target backend may omit these from its build. */
#ifdef MB_HOST_BUILD
void platform_host_set_distance(uint16_t cm, bool valid);
void platform_host_set_sensor_busy(bool busy);
void platform_host_set_sensor_delay(uint32_t delay_ms);
void platform_host_feed_uart(const char *s);
uint16_t platform_host_servo_pulse(mb_wheel_t wheel);
void platform_host_advance_ms(uint32_t ms);
void platform_host_reset(void);
unsigned platform_host_sensor_start_count(void);
#endif
#endif
