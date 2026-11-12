#include "platform.h"
#include "navigation.h"
#include "motor_driver.h"
#include "servo_driver.h"
#include "hc_sr04_driver.h"
#include "obstacle_detection.h"
#include "uart_command.h"
#include "microbot_config.h"
#include <stdio.h>
#include <string.h>
static unsigned passed,failed;
#define CHECK(x) do { if(x) passed++; else { failed++; printf("FAIL %s:%d: %s\n",__FILE__,__LINE__,#x); } } while(0)
static void tick_ms(unsigned ms) { unsigned i; for(i=0;i<ms;i++){ navigation_tick(); platform_idle(); } }
static void reset_all(void) { platform_init(); uart_command_init(); navigation_init(MB_SCENARIO_RECOVERY); }
static void test_servo_motor(void) {
    reset_all(); motor_stop();
    CHECK(platform_host_servo_pulse(MB_LEFT)==1500u);
    motor_forward(30,40);
    CHECK(platform_host_servo_pulse(MB_LEFT)!=1500u);
    CHECK(platform_host_servo_pulse(MB_RIGHT)!=1500u);
    motor_turn_right(35);
    CHECK(platform_host_servo_pulse(MB_LEFT)>1500u);
    CHECK(platform_host_servo_pulse(MB_RIGHT)<1500u);
}
static void test_sensor(void) {
    mb_distance_t r; reset_all();
    platform_host_set_distance(45,true);
    CHECK(obstacle_measure(&r)==MB_PATH_CLEAR && r.distance_cm==45u);
    platform_host_set_distance(20,true);
    CHECK(obstacle_measure(&r)==MB_PATH_BLOCKED);
    platform_host_set_distance(0,false);
    CHECK(obstacle_measure(&r)==MB_PATH_INVALID);
}
static void test_sensor_three_invalid_attempts(void) {
    mb_distance_t result;
    unsigned before;

    reset_all();
    platform_host_set_distance(0u, false);
    before = platform_host_sensor_start_count();

    CHECK(obstacle_measure(&result) == MB_PATH_INVALID);
    CHECK(platform_host_sensor_start_count() - before == MB_SENSOR_RETRIES);
    CHECK(result.valid == false);
    CHECK(result.timeout == true);
}

static void test_sensor_conversion_timeout(void) {
    mb_distance_t result;

    reset_all();
    platform_host_set_distance(50u, true);
    platform_host_set_sensor_delay(
        MB_SENSOR_CONVERSION_TIMEOUT_MS + 10u
    );

    CHECK(hc_sr04_start());

    while (!hc_sr04_poll(&result)) {
        platform_idle();
    }

    CHECK(result.valid == false);
    CHECK(result.timeout == true);
    CHECK(result.distance_cm == 0u);
    CHECK(platform_millis() == MB_SENSOR_CONVERSION_TIMEOUT_MS);
}

static void test_sensor_failure_requires_reset(void) {
    mb_status_t s;

    reset_all();
    navigation_start();
    CHECK(navigation_status().state == MB_STATE_FORWARD);

    platform_host_set_distance(0u, false);
    tick_ms(MB_SENSOR_INTERVAL_MS + 5u);

    s = navigation_status();
    CHECK(s.state == MB_STATE_SAFE_STOP);
    CHECK(s.started == false);
    CHECK(s.sensor_fault == true);
    CHECK(s.stop_latched == true);
    CHECK(platform_host_servo_pulse(MB_LEFT) == MB_SERVO_NEUTRAL_US);
    CHECK(platform_host_servo_pulse(MB_RIGHT) == MB_SERVO_NEUTRAL_US);

    navigation_start();
    CHECK(navigation_status().state == MB_STATE_SAFE_STOP);
    CHECK(navigation_status().started == false);
    CHECK(navigation_status().stop_latched == true);

    navigation_reset();
    CHECK(navigation_status().state == MB_STATE_WAIT_START);
    CHECK(navigation_status().sensor_fault == false);
    CHECK(navigation_status().stop_latched == false);

    navigation_start();
    CHECK(navigation_status().state == MB_STATE_FORWARD);
    CHECK(navigation_status().started == true);
}

static void test_start_and_obstacle(void) {
    mb_status_t s; reset_all();
    CHECK(navigation_status().state==MB_STATE_WAIT_START);
    navigation_start(); CHECK(navigation_status().state==MB_STATE_FORWARD);
    platform_host_set_distance(10,true);
    tick_ms(105);
    s=navigation_status();
    CHECK(s.state==MB_STATE_STOPPING || s.state==MB_STATE_TURN_RIGHT);
}
static void test_uart(void) {
    reset_all();

    /* START: robot begins moving. */
    platform_host_feed_uart("START\r\n");
    uart_command_poll();
    CHECK(navigation_status().state == MB_STATE_FORWARD);
    CHECK(navigation_status().started == true);

    /* STOP: robot stops and latch activates. */
    platform_host_feed_uart("STOP\r\n");
    uart_command_poll();
    CHECK(navigation_status().state == MB_STATE_SAFE_STOP);
    CHECK(navigation_status().started == false);
    CHECK(navigation_status().stop_latched == true);
    CHECK(platform_host_servo_pulse(MB_LEFT) == MB_SERVO_NEUTRAL_US);
    CHECK(platform_host_servo_pulse(MB_RIGHT) == MB_SERVO_NEUTRAL_US);

    /* START without RESET must be rejected. */
    platform_host_feed_uart("START\r\n");
    uart_command_poll();
    CHECK(navigation_status().state == MB_STATE_SAFE_STOP);
    CHECK(navigation_status().started == false);
    CHECK(navigation_status().stop_latched == true);

    /* RESET clears latch but does not start movement. */
    platform_host_feed_uart("RESET\r\n");
    uart_command_poll();
    CHECK(navigation_status().state == MB_STATE_WAIT_START);
    CHECK(navigation_status().started == false);
    CHECK(navigation_status().stop_latched == false);

    /* START after RESET must work. */
    platform_host_feed_uart("START\r\n");
    uart_command_poll();
    CHECK(navigation_status().state == MB_STATE_FORWARD);
    CHECK(navigation_status().started == true);
    CHECK(navigation_status().stop_latched == false);
}

static void test_uart_speed_commands(void) {
    reset_all();

    platform_host_feed_uart("SPEED SLOW\r\n");
    uart_command_poll();
    CHECK(navigation_status().selected_speed == MB_SPEED_SLOW);
    CHECK(navigation_status().left_speed_percent == MB_SPEED_SLOW_DEFAULT);
    CHECK(navigation_status().right_speed_percent == MB_SPEED_SLOW_DEFAULT);

    platform_host_feed_uart("SPEED MEDIUM\r\n");
    uart_command_poll();
    CHECK(navigation_status().selected_speed == MB_SPEED_MEDIUM);
    CHECK(navigation_status().left_speed_percent == MB_SPEED_MEDIUM_DEFAULT);
    CHECK(navigation_status().right_speed_percent == MB_SPEED_MEDIUM_DEFAULT);

    platform_host_feed_uart("SPEED FAST\r\n");
    uart_command_poll();
    CHECK(navigation_status().selected_speed == MB_SPEED_FAST);
    CHECK(navigation_status().left_speed_percent == MB_SPEED_FAST_DEFAULT);
    CHECK(navigation_status().right_speed_percent == MB_SPEED_FAST_DEFAULT);
}

static void test_speed_ramping(void) {
    uint16_t left_initial, right_initial;
    uint16_t left_step1, right_step1;
    uint16_t left_step2, right_step2;

    reset_all();

    navigation_start();

    left_initial = platform_host_servo_pulse(MB_LEFT);
    right_initial = platform_host_servo_pulse(MB_RIGHT);

    CHECK(left_initial == MB_SERVO_NEUTRAL_US);
    CHECK(right_initial == MB_SERVO_NEUTRAL_US);

    platform_host_advance_ms(MB_SPEED_RAMP_STEP_MS);
    navigation_tick();

    left_step1 = platform_host_servo_pulse(MB_LEFT);
    right_step1 = platform_host_servo_pulse(MB_RIGHT);

    CHECK(left_step1 == MB_SERVO_NEUTRAL_US + 16u);
    CHECK(right_step1 == MB_SERVO_NEUTRAL_US + 16u);

    platform_host_advance_ms(MB_SPEED_RAMP_STEP_MS);
    navigation_tick();

    left_step2 = platform_host_servo_pulse(MB_LEFT);
    right_step2 = platform_host_servo_pulse(MB_RIGHT);

    CHECK(left_step2 == MB_SERVO_NEUTRAL_US + 32u);
    CHECK(right_step2 == MB_SERVO_NEUTRAL_US + 32u);

    platform_host_set_distance(50u, true);

    {
        unsigned step;
        unsigned total_steps =
            (MB_SPEED_SLOW_DEFAULT + MB_SPEED_RAMP_STEP_PERCENT - 1u) /
            MB_SPEED_RAMP_STEP_PERCENT;

        for (step = 2u; step < total_steps; step++) {
            platform_host_advance_ms(MB_SPEED_RAMP_STEP_MS);
            navigation_tick();
        }
    }

    CHECK(navigation_status().left_speed_percent ==
          MB_SPEED_SLOW_DEFAULT);
    CHECK(navigation_status().right_speed_percent ==
          MB_SPEED_SLOW_DEFAULT);
}

static void test_speed_change_while_moving(void) {
    unsigned step;

    reset_all();
    navigation_start();

    /* Reach 10% from the initial zero speed. */
    for (step = 0u; step < 5u; step++) {
        platform_host_advance_ms(MB_SPEED_RAMP_STEP_MS);
        navigation_tick();
    }

    CHECK(navigation_status().left_speed_percent == 10u);
    CHECK(navigation_status().right_speed_percent == 10u);

    /* Request FAST while the robot is already moving. */
    platform_host_feed_uart("SPEED FAST\r\n");
    uart_command_poll();

    CHECK(navigation_status().selected_speed == MB_SPEED_FAST);
    CHECK(navigation_status().left_speed_percent == 10u);
    CHECK(navigation_status().right_speed_percent == 10u);

    /* Verify gradual increase from 10% to 65%. */
    for (step = 0u; step < 28u; step++) {
        platform_host_advance_ms(MB_SPEED_RAMP_STEP_MS);
        navigation_tick();

        {
            uint8_t expected =
                (uint8_t)(12u + (step * 2u));

            if (expected > MB_SPEED_FAST_DEFAULT)
                expected = MB_SPEED_FAST_DEFAULT;

            CHECK(navigation_status().left_speed_percent == expected);
            CHECK(navigation_status().right_speed_percent == expected);
        }
    }

    CHECK(navigation_status().left_speed_percent ==
          MB_SPEED_FAST_DEFAULT);
    CHECK(navigation_status().right_speed_percent ==
          MB_SPEED_FAST_DEFAULT);
}

static void test_stop_latch_requires_reset(void) {
    reset_all();

    /* START: navigation begins. */
    navigation_start();
    CHECK(navigation_status().state == MB_STATE_FORWARD);
    CHECK(navigation_status().started == true);

    /* STOP: motors stop and the safety latch activates. */
    navigation_stop();
    CHECK(navigation_status().state == MB_STATE_SAFE_STOP);
    CHECK(navigation_status().started == false);
    CHECK(navigation_status().stop_latched == true);
    CHECK(platform_host_servo_pulse(MB_LEFT) == MB_SERVO_NEUTRAL_US);
    CHECK(platform_host_servo_pulse(MB_RIGHT) == MB_SERVO_NEUTRAL_US);

    /* START without RESET must be rejected. */
    navigation_start();
    CHECK(navigation_status().state == MB_STATE_SAFE_STOP);
    CHECK(navigation_status().started == false);
    CHECK(navigation_status().stop_latched == true);
    CHECK(platform_host_servo_pulse(MB_LEFT) == MB_SERVO_NEUTRAL_US);
    CHECK(platform_host_servo_pulse(MB_RIGHT) == MB_SERVO_NEUTRAL_US);

    /* RESET clears the latch but does not start navigation. */
    navigation_reset();
    CHECK(navigation_status().state == MB_STATE_WAIT_START);
    CHECK(navigation_status().started == false);
    CHECK(navigation_status().stop_latched == false);
    CHECK(platform_host_servo_pulse(MB_LEFT) == MB_SERVO_NEUTRAL_US);
    CHECK(platform_host_servo_pulse(MB_RIGHT) == MB_SERVO_NEUTRAL_US);

    /* A new START after RESET must work. */
    navigation_start();
    CHECK(navigation_status().state == MB_STATE_FORWARD);
    CHECK(navigation_status().started == true);
    CHECK(navigation_status().stop_latched == false);
}

static void test_recovery_success(void) {
    reset_all(); navigation_start();
    platform_host_set_distance(10,true); tick_ms(105);
    tick_ms(400); /* during right turn */
    platform_host_set_distance(80,true);
    tick_ms(120); /* finish turn and recheck clear */
    CHECK(navigation_status().state==MB_STATE_FORWARD);
}

static void test_scenario1_blocked_safe_stop(void) {
    reset_all();
    navigation_init(MB_SCENARIO_BASIC);
    navigation_start();

    platform_host_set_distance(10, true);
    tick_ms(MB_SENSOR_INTERVAL_MS + 5u);

    CHECK(navigation_status().state == MB_STATE_TURN_RIGHT);

    /* Finish the right turn while the path remains blocked. */
    tick_ms(MB_RIGHT_TURN_MS + 2u);

    CHECK(navigation_status().state == MB_STATE_SAFE_STOP);
    CHECK(platform_host_servo_pulse(MB_LEFT) == MB_SERVO_NEUTRAL_US);
    CHECK(platform_host_servo_pulse(MB_RIGHT) == MB_SERVO_NEUTRAL_US);
}

static void test_scenario2_both_turns_blocked(void) {
    reset_all();
    navigation_init(MB_SCENARIO_TURN_SELECT);
    navigation_start();

    platform_host_set_distance(10, true);
    tick_ms(MB_SENSOR_INTERVAL_MS + 5u);

    CHECK(navigation_status().state == MB_STATE_TURN_RIGHT);

    /* Complete right turn; path remains blocked. */
    tick_ms(MB_RIGHT_TURN_MS + 2u);
    CHECK(navigation_status().state == MB_STATE_TURN_LEFT);

    /* Complete left turn; path remains blocked. */
    tick_ms(MB_LEFT_TURN_MS + 2u);
    CHECK(navigation_status().state == MB_STATE_SAFE_STOP);

    CHECK(platform_host_servo_pulse(MB_LEFT) == MB_SERVO_NEUTRAL_US);
    CHECK(platform_host_servo_pulse(MB_RIGHT) == MB_SERVO_NEUTRAL_US);
}

static void test_scenario3_recovery_exhaustion(void) {
    unsigned cycle;

    reset_all();
    navigation_init(MB_SCENARIO_RECOVERY);
    navigation_start();

    platform_host_set_distance(10, true);
    tick_ms(MB_SENSOR_INTERVAL_MS + 5u);

    CHECK(navigation_status().state == MB_STATE_TURN_RIGHT);
    CHECK(navigation_status().recovery_cycle == 1u);

    for (cycle = 0; cycle < MB_MAX_RECOVERY_CYCLES; cycle++) {
        tick_ms(MB_RIGHT_TURN_MS + 2u);
        CHECK(navigation_status().state == MB_STATE_TURN_LEFT);

        tick_ms(MB_LEFT_TURN_MS + 2u);

        if (cycle == 0u) {
            CHECK(navigation_status().state == MB_STATE_REVERSE);
        } else {
            CHECK(navigation_status().state == MB_STATE_REVERSE);
        }

        /* Advance beyond the reverse limit, then process exactly one tick. */
        platform_host_advance_ms(MB_REVERSE_MAX_MS + 2u);
        navigation_tick();
        CHECK(navigation_status().state == MB_STATE_RECHECK);

        /* Process the recheck separately. */
        navigation_tick();

        if (cycle == 0u) {
            CHECK(navigation_status().state == MB_STATE_TURN_RIGHT);
            CHECK(navigation_status().recovery_cycle == 2u);
        } else {
            CHECK(navigation_status().state == MB_STATE_SAFE_STOP);
        }
    }

    CHECK(platform_host_servo_pulse(MB_LEFT) == MB_SERVO_NEUTRAL_US);
    CHECK(platform_host_servo_pulse(MB_RIGHT) == MB_SERVO_NEUTRAL_US);
}

static void drive_to_reverse(void) {
    reset_all();
    navigation_start();

    platform_host_set_distance(10,true);
    tick_ms(105);

    tick_ms(MB_RIGHT_TURN_MS + 1u);

    tick_ms(MB_LEFT_TURN_MS + 1u);

    navigation_tick();

    CHECK(navigation_status().state==MB_STATE_REVERSE);
}

static void test_reverse_timing(void) {
    unsigned count;
    uint32_t deadline;
    uint32_t reverse_start;

    drive_to_reverse();
    count = platform_host_sensor_start_count();
    deadline = navigation_host_next_sensor_deadline();
    reverse_start = platform_millis() -
        (MB_SENSOR_INTERVAL_MS -
         (deadline - (platform_millis() - MB_SENSOR_INTERVAL_MS)));

    /* No measurement immediately before the scheduled deadline. */
    if ((int32_t)(deadline - platform_millis()) > 0) {
        platform_host_advance_ms(deadline - platform_millis() - 1u);
        navigation_tick();
        CHECK(platform_host_sensor_start_count() == count);
        CHECK(navigation_status().state == MB_STATE_REVERSE);

        platform_host_advance_ms(1u);
        navigation_tick();
        CHECK(platform_host_sensor_start_count() == count + 1u);
    } else {
        CHECK(false);
    }

    /* No second measurement before the next scheduled deadline. */
    platform_host_advance_ms(MB_SENSOR_INTERVAL_MS - 1u);
    navigation_tick();
    CHECK(platform_host_sensor_start_count() == count + 1u);

    platform_host_advance_ms(1u);
    navigation_tick();
    CHECK(platform_host_sensor_start_count() == count + 2u);

    /* Reverse must stop at its 300 ms maximum. */
    (void)reverse_start;
    platform_host_advance_ms(MB_REVERSE_MAX_MS);
    navigation_tick();
    CHECK(navigation_status().state == MB_STATE_RECHECK);
    CHECK(platform_host_servo_pulse(MB_LEFT) == MB_SERVO_NEUTRAL_US);
    CHECK(platform_host_servo_pulse(MB_RIGHT) == MB_SERVO_NEUTRAL_US);
}

static void test_reverse_early_clear(void) {
    unsigned count;

    drive_to_reverse();
    count=platform_host_sensor_start_count();

    platform_host_set_distance(80,true);
    platform_host_advance_ms(MB_SENSOR_INTERVAL_MS);
    navigation_tick();

    CHECK(platform_host_sensor_start_count()==count+1u);
    CHECK(navigation_status().state==MB_STATE_RECHECK);

    /* The next navigation tick processes the clear-path result. */
    navigation_tick();
    CHECK(navigation_status().state==MB_STATE_FORWARD);
}

int main(void) {
    test_servo_motor(); test_sensor();
    test_sensor_three_invalid_attempts();
    test_sensor_conversion_timeout();
    test_sensor_failure_requires_reset();
    test_start_and_obstacle(); test_uart();
    test_uart_speed_commands();
    test_speed_ramping();
    test_speed_change_while_moving();
    test_stop_latch_requires_reset();
    test_recovery_success();
    test_scenario1_blocked_safe_stop();
    test_scenario2_both_turns_blocked();
    test_scenario3_recovery_exhaustion();
    test_reverse_timing(); test_reverse_early_clear();
    printf("MicroBot host tests: %u passed, %u failed\n",passed,failed);
    return failed?1:0;
}
