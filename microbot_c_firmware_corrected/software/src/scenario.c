#include "scenario.h"
#ifndef MB_SCENARIO
#define MB_SCENARIO 3
#endif
mb_scenario_t scenario_selected(void) {
#if MB_SCENARIO == 1
    return MB_SCENARIO_BASIC;
#elif MB_SCENARIO == 2
    return MB_SCENARIO_TURN_SELECT;
#else
    return MB_SCENARIO_RECOVERY;
#endif
}
