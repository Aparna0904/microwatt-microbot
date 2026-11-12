#include "uart_command.h"
#include "platform.h"
#include "navigation.h"
#include "microbot_config.h"
#include <string.h>
#include <stdio.h>
static char line[MB_UART_LINE_MAX];
static unsigned used;
static bool detailed=MB_UART_DEFAULT_DETAILED!=0;
static uint32_t last_telemetry_ms;
void uart_debug_event(const char *e) {
    char b[MB_UART_LINE_MAX+8];
    if(!e) return;
    (void)snprintf(b,sizeof b,"EVENT %s\r\n",e); platform_uart_write(b);
}
void uart_debug_status(void) {
    mb_status_t s=navigation_status();
    char b[128];
    (void)snprintf(b,sizeof b,"STATE=%u DIST_CM=%u LEFT=%u RIGHT=%u RECOVERY=%u SENSOR_FAULT=%u\r\n",
        (unsigned)s.state,(unsigned)s.last_distance_cm,(unsigned)s.left_speed_percent,
        (unsigned)s.right_speed_percent,(unsigned)s.recovery_cycle,s.sensor_fault?1u:0u);
    platform_uart_write(b);
}
static void execute(const char *cmd) {
    if(strcmp(cmd,"START")==0) navigation_start();
    else if(strcmp(cmd,"STOP")==0) navigation_stop();
    else if(strcmp(cmd,"STATUS")==0) uart_debug_status();
    else if(strcmp(cmd,"SPEED SLOW")==0) navigation_set_speed(MB_SPEED_SLOW);
    else if(strcmp(cmd,"SPEED MEDIUM")==0) navigation_set_speed(MB_SPEED_MEDIUM);
    else if(strcmp(cmd,"SPEED FAST")==0) navigation_set_speed(MB_SPEED_FAST);
    else if(strcmp(cmd,"RESET")==0) navigation_reset();
    else if(strcmp(cmd,"DEBUG DETAILED")==0) { detailed=true; uart_debug_event("Detailed debug enabled"); }
    else if(strcmp(cmd,"DEBUG EVENT")==0) { detailed=false; uart_debug_event("Event-only debug enabled"); }
    else uart_debug_event("Invalid UART command");
}
void uart_command_init(void) { used=0; line[0]='\0'; last_telemetry_ms=platform_millis(); }
void uart_command_poll(void) {
    uint8_t c;
    while(platform_uart_read_byte(&c)) {
        if(c=='\r' || c=='\n') {
            if(used>0u) { line[used]='\0'; execute(line); used=0; }
        } else if(used<sizeof(line)-1u) line[used++]=(char)c;
        else { used=0; uart_debug_event("UART command too long"); }
    }
    if(detailed && (uint32_t)(platform_millis()-last_telemetry_ms)>=500u) {
        last_telemetry_ms=platform_millis();
        uart_debug_status();
    }
}
