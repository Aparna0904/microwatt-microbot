#include "navigation.h"
#include "microbot_config.h"
#include "motor_driver.h"
#include "obstacle_detection.h"
#include "hc_sr04_driver.h"
#include "platform.h"
#include "uart_command.h"

static mb_status_t st;
static mb_scenario_t scenario;
static uint32_t state_since, next_sensor, ramp_since;
static uint8_t target_l,target_r,current_l,current_r;
static uint8_t phase; /* 0 right, 1 left, 2 reverse */

static uint32_t elapsed(void) { return platform_millis()-state_since; }
static void set_state(mb_state_t s) { st.state=s; state_since=platform_millis(); }
static void safe_stop(const char *why) {
    motor_stop();
    st.started=false;
    if(MB_STOP_LATCHED_DEFAULT) st.stop_latched=true;
    set_state(MB_STATE_SAFE_STOP);
    uart_debug_event(why);
}
static uint8_t speed_default(mb_speed_t s) {
    if(s==MB_SPEED_SLOW) return MB_SPEED_SLOW_DEFAULT;
    if(s==MB_SPEED_MEDIUM) return MB_SPEED_MEDIUM_DEFAULT;
    return MB_SPEED_FAST_DEFAULT;
}
static void ramp_tick(void) {
    uint32_t now=platform_millis();
    if(now-ramp_since<MB_SPEED_RAMP_STEP_MS) return;
    ramp_since=now;
    if(current_l<target_l) current_l=(uint8_t)(((uint8_t)(target_l-current_l)>MB_SPEED_RAMP_STEP_PERCENT)?current_l+MB_SPEED_RAMP_STEP_PERCENT:target_l);
    else if(current_l>target_l) current_l=(uint8_t)(((uint8_t)(current_l-target_l)>MB_SPEED_RAMP_STEP_PERCENT)?current_l-MB_SPEED_RAMP_STEP_PERCENT:target_l);
    if(current_r<target_r) current_r=(uint8_t)(((uint8_t)(target_r-current_r)>MB_SPEED_RAMP_STEP_PERCENT)?current_r+MB_SPEED_RAMP_STEP_PERCENT:target_r);
    else if(current_r>target_r) current_r=(uint8_t)(((uint8_t)(current_r-target_r)>MB_SPEED_RAMP_STEP_PERCENT)?current_r-MB_SPEED_RAMP_STEP_PERCENT:target_r);
    motor_forward(current_l,current_r);
    st.left_speed_percent=current_l; st.right_speed_percent=current_r;
}
static mb_path_t check_path(void) {
    mb_distance_t r;
    mb_path_t p=obstacle_measure(&r);
    if(p==MB_PATH_INVALID) { st.sensor_fault=true; return p; }
    st.sensor_fault=false; st.last_distance_cm=r.distance_cm; return p;
}
static void begin_cycle(void) {
    phase=0; motor_stop(); set_state(MB_STATE_TURN_RIGHT);
    motor_turn_right(MB_TURN_SPEED_DEFAULT); uart_debug_event("Right turn started");
}
static void resume_forward(void) {
    st.recovery_cycle=0; current_l=0; current_r=0;
    target_l=st.left_speed_percent?st.left_speed_percent:speed_default(st.selected_speed);
    target_r=st.right_speed_percent?st.right_speed_percent:speed_default(st.selected_speed);
    if(target_l==0) target_l=speed_default(st.selected_speed);
    if(target_r==0) target_r=speed_default(st.selected_speed);
    ramp_since=platform_millis(); motor_forward(0,0);
    next_sensor=platform_millis()+MB_SENSOR_INTERVAL_MS;
    set_state(MB_STATE_FORWARD); uart_debug_event("Path clear; resuming slowly");
}
void navigation_init(mb_scenario_t sc) {
    scenario=sc; st.state=MB_STATE_INIT; st.selected_speed=MB_SPEED_SLOW;
    st.left_speed_percent=speed_default(MB_SPEED_SLOW);
    st.right_speed_percent=speed_default(MB_SPEED_SLOW);
    st.recovery_cycle=0; st.started=false; st.sensor_fault=false; st.stop_latched=false;
    current_l=target_l=st.left_speed_percent; current_r=target_r=st.right_speed_percent;
    motor_init(); motor_stop(); hc_sr04_init(); set_state(MB_STATE_WAIT_START);
    uart_debug_event("Initialized; waiting for START");
}
void navigation_start(void) {
    if(st.stop_latched) return;
    st.started=true; current_l=0; current_r=0;
    target_l=st.left_speed_percent; target_r=st.right_speed_percent;
    ramp_since=platform_millis(); next_sensor=platform_millis()+MB_SENSOR_INTERVAL_MS;
    motor_forward(0,0); set_state(MB_STATE_FORWARD); uart_debug_event("START received");
}
void navigation_stop(void) {
    motor_stop(); st.started=false;
    if(MB_STOP_LATCHED_DEFAULT) st.stop_latched=true;
    set_state(MB_STATE_SAFE_STOP); uart_debug_event("STOP received");
}
void navigation_reset(void) {
    motor_stop(); st.started=false; st.stop_latched=false; st.sensor_fault=false;
    st.recovery_cycle=0; current_l=current_r=0; set_state(MB_STATE_WAIT_START);
    uart_debug_event("Navigation reset; waiting for START");
}
void navigation_set_speed(mb_speed_t s) {
    if(s>MB_SPEED_FAST) return;

    st.selected_speed=s;
    target_l=speed_default(s);
    target_r=speed_default(s);

    if(!st.started) {
        st.left_speed_percent=target_l;
        st.right_speed_percent=target_r;
    }
}
void navigation_set_wheel_speeds(uint8_t l,uint8_t r) {
    if(l>65u) l=65u;
    if(r>65u) r=65u;
    st.left_speed_percent=l; st.right_speed_percent=r; target_l=l; target_r=r;
}
void navigation_tick(void) {
    uint32_t now=platform_millis();
    mb_path_t p;
    if(st.state==MB_STATE_WAIT_START || st.state==MB_STATE_SAFE_STOP || st.state==MB_STATE_SENSOR_FAULT) return;
    if(st.state==MB_STATE_FORWARD) {
        ramp_tick();
        if((int32_t)(now-next_sensor)>=0) {
            next_sensor=now+MB_SENSOR_INTERVAL_MS;
            p=check_path();
            if(p==MB_PATH_INVALID) { safe_stop("Sensor invalid after three attempts; safe stop"); return; }
            if(p==MB_PATH_BLOCKED) {
                motor_stop(); st.recovery_cycle=1; phase=0;
                set_state(MB_STATE_STOPPING); uart_debug_event("Obstacle detected");
            }
        }
        return;
    }
    if(st.state==MB_STATE_STOPPING) {
        if(scenario==MB_SCENARIO_BASIC) {
            motor_turn_right(MB_TURN_SPEED_DEFAULT); set_state(MB_STATE_TURN_RIGHT);
        } else begin_cycle();
        return;
    }
    if(st.state==MB_STATE_TURN_RIGHT || st.state==MB_STATE_TURN_LEFT) {
        uint32_t duration=(st.state==MB_STATE_TURN_RIGHT)?MB_RIGHT_TURN_MS:MB_LEFT_TURN_MS;
        if(elapsed()>=duration) { motor_stop(); set_state(MB_STATE_RECHECK); }
        return;
    }
    if(st.state==MB_STATE_RECHECK) {
        p=check_path();
        if(p==MB_PATH_INVALID) { safe_stop("Sensor failure during recovery"); return; }
        if(p==MB_PATH_CLEAR) { resume_forward(); return; }
        if(scenario==MB_SCENARIO_BASIC) { safe_stop("Basic scenario: path still blocked"); return; }
        if(phase==0u) {
            phase=1u; motor_turn_left(MB_TURN_SPEED_DEFAULT); set_state(MB_STATE_TURN_LEFT);
            uart_debug_event("Left alternative turn started"); return;
        }
        if(scenario==MB_SCENARIO_TURN_SELECT) { safe_stop("Scenario 2: both turns blocked"); return; }
        if(phase==2u) {
            if(st.recovery_cycle < MB_MAX_RECOVERY_CYCLES) {
                st.recovery_cycle++;
                begin_cycle();
            } else {
                safe_stop("Both recovery cycles failed");
            }
            return;
        }
        phase=2u; motor_reverse(MB_REVERSE_SPEED_DEFAULT,MB_REVERSE_SPEED_DEFAULT);
        next_sensor=platform_millis()+MB_SENSOR_INTERVAL_MS;
        set_state(MB_STATE_REVERSE); uart_debug_event("Reverse recovery started"); return;
    }
    if(st.state==MB_STATE_REVERSE) {
        /* Stop reversing when the maximum duration is reached. */
        if(elapsed()>=MB_REVERSE_MAX_MS) {
            motor_stop();
            set_state(MB_STATE_RECHECK);
            return;
        }

        /* Check the forward-facing sensor at the scheduled deadline. */
        if((int32_t)(now-next_sensor)>=0) {
            next_sensor=now+MB_SENSOR_INTERVAL_MS;

            p=check_path();

            if(p==MB_PATH_INVALID) {
                safe_stop("Sensor failure while reversing");
                return;
            }

            if(p==MB_PATH_CLEAR) {
                motor_stop();
                set_state(MB_STATE_RECHECK);
                return;
            }
        }
        return;
    }
}
mb_status_t navigation_status(void) { return st; }

#ifdef MB_HOST_BUILD
uint32_t navigation_host_next_sensor_deadline(void) {
    return next_sensor;
}
#endif
