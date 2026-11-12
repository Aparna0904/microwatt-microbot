#include "platform.h"
#include <stddef.h>
#define UART_BUF 256u
static uint32_t now_ms;
static uint16_t sensor_cm=100u, pulses[2]={1500u,1500u};
static bool sensor_valid=true, sensor_busy=false;
static unsigned sensor_start_count;
static uint32_t sensor_delay_ms;
static uint32_t sensor_started_ms;
static char uart_in[UART_BUF];
static unsigned uart_head, uart_tail;
void platform_host_reset(void) {
    now_ms=0; sensor_cm=100; sensor_valid=true; sensor_busy=false;
    sensor_start_count=0; sensor_delay_ms=0; sensor_started_ms=0;
    pulses[0]=pulses[1]=1500; uart_head=uart_tail=0;
}
void platform_init(void) { platform_host_reset(); }
uint32_t platform_millis(void) { return now_ms; }
void platform_idle(void) { now_ms++; }
bool platform_sensor_start(void) {
    if(sensor_busy) return false;
    sensor_busy=true;
    sensor_started_ms=now_ms;
    sensor_start_count++;
    return true;
}
bool platform_sensor_result(mb_distance_t *out) {
    if(!out || !sensor_busy) return false;
    if((uint32_t)(now_ms-sensor_started_ms)<sensor_delay_ms) return false;
    sensor_busy=false; out->distance_cm=sensor_cm; out->valid=sensor_valid; out->timeout=false;
    return true;
}
void platform_servo_set_pulse(mb_wheel_t w,uint16_t p) { if((unsigned)w<2u) pulses[w]=p; }
bool platform_uart_read_byte(uint8_t *out) {
    if(!out || uart_tail==uart_head) return false;
    *out=(uint8_t)uart_in[uart_tail++ % UART_BUF]; return true;
}
void platform_uart_write(const char *s) { (void)s; }
void platform_host_set_distance(uint16_t cm,bool valid) { sensor_cm=cm; sensor_valid=valid; }
void platform_host_set_sensor_busy(bool busy) { sensor_busy=busy; }
void platform_host_set_sensor_delay(uint32_t delay_ms) {
    sensor_delay_ms=delay_ms;
}

void platform_host_feed_uart(const char *s) {
    if(!s) return;
    while(*s && (uart_head-uart_tail)<UART_BUF) uart_in[uart_head++ % UART_BUF]=*s++;
}
uint16_t platform_host_servo_pulse(mb_wheel_t w) { return ((unsigned)w<2u)?pulses[w]:0u; }
void platform_host_advance_ms(uint32_t ms) { now_ms+=ms; }

unsigned platform_host_sensor_start_count(void) {
    return sensor_start_count;
}
