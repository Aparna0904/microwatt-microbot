#ifndef MICROB0T_UART_COMMAND_H
#define MICROB0T_UART_COMMAND_H
void uart_command_init(void);
void uart_command_poll(void);
void uart_debug_event(const char *event);
void uart_debug_status(void);
#endif
