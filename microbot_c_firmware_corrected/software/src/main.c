#include "platform.h"
#include "navigation.h"
#include "uart_command.h"
#include "scenario.h"
int main(void) {
    platform_init();
    uart_command_init();
    navigation_init(scenario_selected());
    for(;;) {
        uart_command_poll(); /* STOP is processed before navigation advances. */
        navigation_tick();
        platform_idle();
    }
}
