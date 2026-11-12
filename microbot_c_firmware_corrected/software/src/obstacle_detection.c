#include "obstacle_detection.h"
#include "hc_sr04_driver.h"
#include "platform.h"
#include "microbot_config.h"
mb_path_t obstacle_measure(mb_distance_t *out) {
    unsigned attempt;
    if(!out) return MB_PATH_INVALID;
    for(attempt=0;attempt<MB_SENSOR_RETRIES;attempt++) {
        uint32_t start=platform_millis();
        mb_distance_t r={0,false,false};
        if(!hc_sr04_start()) { platform_idle(); continue; }
        while(!hc_sr04_poll(&r)) {
            if((uint32_t)(platform_millis()-start)>=MB_SENSOR_CONVERSION_TIMEOUT_MS) break;
            platform_idle();
        }
        if(r.valid) { *out=r; return r.distance_cm<=MB_OBSTACLE_THRESHOLD_CM?MB_PATH_BLOCKED:MB_PATH_CLEAR; }
    }
    out->valid=false; out->timeout=true; out->distance_cm=0;
    return MB_PATH_INVALID;
}
