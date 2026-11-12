#include "servo_driver.h"
#include "platform.h"
#include "microbot_config.h"
static mb_servo_cal_t cal[2] = {
    {MB_SERVO_NEUTRAL_US, MB_SERVO_MIN_PULSE_US, MB_SERVO_MAX_PULSE_US, 1u, 65u},
    {MB_SERVO_NEUTRAL_US, MB_SERVO_MIN_PULSE_US, MB_SERVO_MAX_PULSE_US, 1u, 65u}
};
static uint8_t clamp_percent(uint8_t p) { return p>100u?100u:p; }
void servo_init(void) { servo_stop_all(); }
void servo_set_calibration(mb_wheel_t w,const mb_servo_cal_t *c) {
    if((unsigned)w>=2u || !c) return;
    if(c->min_us>=c->neutral_us || c->neutral_us>=c->max_us ||
       c->min_us<MB_SERVO_MIN_PULSE_US || c->max_us>MB_SERVO_MAX_PULSE_US ||
       (c->forward_sign!=1u && c->forward_sign!=0u)) return;
    cal[w]=*c;
}
uint16_t servo_pulse_for(mb_wheel_t w,mb_direction_t d,uint8_t speed) {
    uint32_t span, delta; int32_t sign; uint32_t pulse;
    if((unsigned)w>=2u || d==MB_DIR_STOP || speed==0u) return (unsigned)w<2u?cal[w].neutral_us:MB_SERVO_NEUTRAL_US;
    speed=clamp_percent(speed);
    if(speed>cal[w].max_speed_percent) speed=cal[w].max_speed_percent;
    /* Calibrated forward direction chooses which side of neutral is forward. */
    sign=((d==MB_DIR_FORWARD) == (cal[w].forward_sign!=0u)) ? 1 : -1;
    span=(sign>0)?(uint32_t)(cal[w].max_us-cal[w].neutral_us):(uint32_t)(cal[w].neutral_us-cal[w].min_us);
    delta=(span*speed)/100u;
    pulse=(sign>0)?(uint32_t)cal[w].neutral_us+delta:(uint32_t)cal[w].neutral_us-delta;
    if(pulse>cal[w].max_us) pulse=cal[w].max_us;
    if(pulse<cal[w].min_us) pulse=cal[w].min_us;
    return (uint16_t)pulse;
}
void servo_command(mb_wheel_t w,mb_direction_t d,uint8_t speed) {
    if((unsigned)w<2u) platform_servo_set_pulse(w,servo_pulse_for(w,d,speed));
}
void servo_stop_all(void) {
    platform_servo_set_pulse(MB_LEFT,cal[MB_LEFT].neutral_us);
    platform_servo_set_pulse(MB_RIGHT,cal[MB_RIGHT].neutral_us);
}
