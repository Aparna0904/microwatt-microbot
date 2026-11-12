#include "motor_driver.h"
#include "servo_driver.h"
void motor_init(void) { servo_init(); }
void motor_stop(void) { servo_stop_all(); }
void motor_set_wheels(mb_direction_t ld,uint8_t ls,mb_direction_t rd,uint8_t rs) {
    servo_command(MB_LEFT,ld,ls); servo_command(MB_RIGHT,rd,rs);
}
void motor_forward(uint8_t l,uint8_t r) { motor_set_wheels(MB_DIR_FORWARD,l,MB_DIR_FORWARD,r); }
void motor_reverse(uint8_t l,uint8_t r) { motor_set_wheels(MB_DIR_REVERSE,l,MB_DIR_REVERSE,r); }
void motor_turn_right(uint8_t s) { motor_set_wheels(MB_DIR_FORWARD,s,MB_DIR_REVERSE,s); }
void motor_turn_left(uint8_t s) { motor_set_wheels(MB_DIR_REVERSE,s,MB_DIR_FORWARD,s); }
