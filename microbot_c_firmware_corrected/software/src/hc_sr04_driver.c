#include "hc_sr04_driver.h"
#include "platform.h"
#include "microbot_config.h"
static bool active;
static uint32_t started_ms;
void hc_sr04_init(void) { active=false; }
bool hc_sr04_start(void) {
    if(active) return false;
    if(!platform_sensor_start()) return false;
    active=true; started_ms=platform_millis(); return true;
}
bool hc_sr04_poll(mb_distance_t *out) {
    if(!out || !active) return false;
    if(platform_sensor_result(out)) {
        active=false;
        if(out->distance_cm==0u || out->distance_cm>400u) out->valid=false;
        return true;
    }
    if((uint32_t)(platform_millis()-started_ms)>=MB_SENSOR_CONVERSION_TIMEOUT_MS) {
        active=false; out->valid=false; out->timeout=true; out->distance_cm=0; return true;
    }
    return false; /* measurement is still in progress */
}
