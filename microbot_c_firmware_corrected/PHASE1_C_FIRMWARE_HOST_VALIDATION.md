# Phase 1 – MicroBot C Firmware Development and Host Validation

## Document purpose

This document explains the work completed in Phase 1 of the MicroBot project. It is written for readers who have not previously worked on the project. It describes the firmware architecture, the operation of each software module, robot movement and recovery logic, safety behaviour, host-side testing, observed results, and work remaining before FPGA and physical-robot validation.

> **Scope:** Phase 1 covers C firmware development and host-side validation. The results are from the host GCC test environment. They do not demonstrate a successful Microwatt target build, FPGA implementation, or physical robot test.

## 1. Project overview and objective

MicroBot is intended to use an OpenPOWER Microwatt processor implemented on a Digilent Arty A7-100T FPGA. The planned robot hardware consists of two FS5106R continuous-rotation servo motors, one forward-facing HC-SR04 ultrasonic distance sensor, UART communication, and custom VHDL peripherals connected through a Wishbone interface.

The objective of Phase 1 was to develop the C software that makes movement decisions and controls these devices. The C control logic was developed before the final VHDL peripherals and their Wishbone register map were available. The firmware therefore uses a Hardware Abstraction Layer (HAL): application code calls common platform functions instead of directly accessing hardware registers.

This allowed the C logic to be compiled and tested on a development computer using a simulated platform. It also avoids putting guessed hardware addresses into the firmware.

### 1.1 Intended robot operation

1. The firmware initializes the selected platform, UART command handling, motor driver, sensor driver, and navigation state.
2. The robot enters `MB_STATE_WAIT_START` and remains stopped until it receives a `START` command.
3. When START is accepted, navigation enters `MB_STATE_FORWARD` and gradually ramps the wheel-speed commands towards the selected target.
4. While moving, navigation checks the forward-facing ultrasonic sensor at the configured interval.
5. A valid distance greater than the obstacle threshold is classified as clear. A distance at or below the threshold is classified as blocked.
6. A blocked path starts the selected obstacle-recovery scenario.
7. If the sensor cannot provide a valid measurement after the configured retries, the firmware stops the motors and enters a latched safety-stop condition.
8. A latched safety stop cannot be cleared by START. The operator must issue RESET. The robot then returns to WAIT_START and requires a new START.

## 2. Target hardware and software boundary

| Item | Intended platform/device | Role |
|---|---|---|
| Processor | OpenPOWER Microwatt | Executes the C firmware |
| FPGA | Digilent Arty A7-100T | Hosts Microwatt and custom peripherals |
| Left actuator | FS5106R continuous-rotation servo | Drives the left wheel |
| Right actuator | FS5106R continuous-rotation servo | Drives the right wheel |
| Distance sensor | HC-SR04 | Measures forward obstacle distance |
| Communication | UART | Receives commands and reports status |
| Peripheral bus | Wishbone | Connects the processor to custom peripherals |

The actual VHDL peripherals, register addresses, and target platform bindings are not completed in Phase 1. They are later integration work.

### 2.1 Software boundary

The firmware is separated from the hardware implementation:

- Navigation and application code decide what the robot should do.
- Device drivers express operations such as moving, stopping, measuring distance, and setting servo pulses.
- The platform layer provides the low-level functions used by the drivers.
- The host platform implements those functions for software tests.
- The Microwatt platform file is an integration template that must be completed using the confirmed hardware interface.

This separation allows the same navigation logic to be tested with a host mock before it is connected to actual FPGA peripherals.

## 3. Software architecture and file responsibilities

The firmware source is organized under `software/`.

### 3.1 Header files – `software/include/`

| Header | Responsibility |
|---|---|
| `microbot_config.h` | Central configuration constants, thresholds, timings, speed values, and feature settings |
| `microbot_types.h` | Shared enumerations, structures, and data types |
| `platform.h` | Common platform abstraction interface and host-test declarations |
| `servo_driver.h` | Servo driver interface |
| `motor_driver.h` | Higher-level motor movement interface |
| `hc_sr04_driver.h` | Ultrasonic sensor driver interface |
| `obstacle_detection.h` | Path classification interface |
| `navigation.h` | Navigation initialization, commands, state updates, and status interface |
| `uart_command.h` | UART command-handler interface |
| `scenario.h` | Navigation scenario selection and interface |

### 3.2 Source files – `software/src/`

| Source file | Responsibility and operation |
|---|---|
| `main.c` | Initializes the platform, UART command handler, and navigation; then repeatedly polls UART, updates navigation, and calls platform idle |
| `platform.c` | Selects the platform implementation according to explicit build configuration |
| `platform_host.c` | Supplies simulated time, sensor, UART, and servo behaviour for host testing |
| `platform_microwatt.c` | Contains the not-yet-finalized Microwatt backend; it deliberately prevents use of an unconfirmed register map |
| `servo_driver.c` | Converts requested wheel speed and direction into servo pulse-width commands using configured calibration |
| `motor_driver.c` | Provides forward, stop, turn, and reverse operations using the servo driver |
| `hc_sr04_driver.c` | Starts ultrasonic measurements, polls for completion, applies the conversion timeout, and reports valid or invalid results |
| `obstacle_detection.c` | Interprets sensor results and classifies the path as clear, blocked, or invalid |
| `navigation.c` | Implements the robot state machine, speed ramping, periodic sensor checks, obstacle recovery, and safety-stop handling |
| `uart_command.c` | Receives and interprets text commands such as START, STOP, STATUS, SPEED, and RESET |
| `scenario.c` | Selects and provides the configured navigation scenario |

### 3.3 Test and build files

- `software/tests/test_main.c` contains the host-side test assertions.
- `software/Makefile` contains the host build and test rules.
- Supporting documentation includes `BUILD_INSTRUCTIONS.md`, `SOFTWARE_ARCHITECTURE.md`, `HARDWARE_INTERFACE_REQUIREMENTS.md`, `TEST_PLAN.md`, and `UART_COMMAND_REFERENCE.md`.

## 4. Main program operation

The application entry point is `software/src/main.c`.

The initialization order is:

1. `platform_init()` initializes the selected platform.
2. `uart_command_init()` initializes command reception and parsing.
3. `navigation_init(scenario_selected())` initializes navigation using the selected scenario.

The firmware then runs a continuous loop:

```c
for (;;) {
    uart_command_poll();
    navigation_tick();
    platform_idle();
}
```

Each call has a distinct purpose:

- `uart_command_poll()` checks whether a complete UART command has arrived and processes it.
- `navigation_tick()` advances the navigation state machine. It checks timers, updates the speed ramp, checks the sensor when due, and progresses obstacle recovery.
- `platform_idle()` gives the selected platform an opportunity to perform its idle or time-advancement operation. In the host backend this supports deterministic simulated time.

The loop repeatedly services command handling and navigation instead of implementing the entire robot behaviour as one long blocking operation.

## 5. Configuration values

The firmware centralizes operating values in `software/include/microbot_config.h`. The following are the configured values in the current firmware baseline.

| Configuration | Value | Meaning |
|---|---:|---|
| `MB_OBSTACLE_THRESHOLD_CM` | 20 cm | Distance at or below which the path is blocked |
| `MB_SENSOR_INTERVAL_MS` | 100 ms | Interval between scheduled forward-path checks |
| `MB_SENSOR_RETRIES` | 3 | Measurement attempts when readings are invalid |
| `MB_SENSOR_CONVERSION_TIMEOUT_MS` | 40 ms | Maximum wait for one sensor conversion |
| `MB_RIGHT_TURN_MS` | 500 ms | Configured right-turn duration |
| `MB_LEFT_TURN_MS` | 500 ms | Configured left-turn duration |
| `MB_REVERSE_MAX_MS` | 300 ms | Maximum configured reverse duration |
| `MB_MAX_RECOVERY_CYCLES` | 2 | Maximum recovery cycles in the recovery scenario |
| `MB_SPEED_RAMP_STEP_MS` | 50 ms | Speed-ramp update interval |
| `MB_SPEED_RAMP_STEP_PERCENT` | 2 percentage points | Maximum speed change per ramp update |
| `MB_SPEED_SLOW_DEFAULT` | 25% | SLOW target |
| `MB_SPEED_MEDIUM_DEFAULT` | 45% | MEDIUM target |
| `MB_SPEED_FAST_DEFAULT` | 65% | FAST target |
| `MB_TURN_SPEED_DEFAULT` | 35% | Configured turning speed |
| `MB_REVERSE_SPEED_DEFAULT` | 30% | Configured reverse speed |
| `MB_SERVO_PERIOD_US` | 20,000 us | Nominal servo control period |
| `MB_SERVO_NEUTRAL_US` | 1,500 us | Configured neutral pulse |
| `MB_SERVO_MIN_PULSE_US` | 700 us | Configured minimum pulse |
| `MB_SERVO_MAX_PULSE_US` | 2,300 us | Configured maximum pulse |
| `MB_STOP_LATCHED_DEFAULT` | 1 | Enables software stop-latch behaviour |

These are firmware configuration values, not measurements from the physical robot. Servo neutral points, direction, and pulse limits must be calibrated on the actual FS5106R units.

## 6. Hardware Abstraction Layer

### 6.1 Why the HAL is used

The final C-to-hardware interface depends on VHDL peripherals and a Wishbone register map. At the time of Phase 1, those register definitions were not finalized.

The HAL prevents navigation code from depending on unverified register addresses. The application uses a common platform interface, and a selected backend supplies the implementation.

### 6.2 Platform files

- **`platform.h`** declares the platform functions used by drivers and the application.
- **`platform.c`** selects the implementation. The host build is explicitly selected with `MB_HOST_BUILD=1`; the host implementation is not silently assumed for a target build.
- **`platform_host.c`** implements a mock platform for tests. It simulates elapsed time, sensor results and delays, UART input, and servo pulse outputs. Test helpers can set a distance and validity, configure sensor delay or busy condition, feed UART text, advance simulated time, reset the mock, read servo outputs, and count sensor starts.
- **`platform_microwatt.c`** is deliberately not a finished target backend. It contains fail-closed integration guards so an incomplete or guessed register map cannot be mistaken for a working Microwatt implementation.

### 6.3 What host simulation can and cannot prove

Host simulation can verify C control flow, state transitions, command handling, timeout and retry logic, and the pulse values requested by the software mock.

It cannot verify actual Wishbone transactions, VHDL timing, FPGA pin assignments, electrical voltage levels, physical servo calibration, or actual robot movement.

## 7. Servo and motor operation

### 7.1 Continuous-rotation servo principle

A continuous-rotation servo uses the control pulse to command direction and speed. It is not commanded to move to a particular angular position like a positional servo.

The configured control period is 20 ms (50 Hz nominal), and the configured neutral pulse is 1,500 us. Pulses on either side of the calibrated neutral point command rotation in opposite directions. The exact direction and speed response depend on physical servo calibration.

### 7.2 Servo driver

`servo_driver.c` converts a requested speed percentage and direction into a pulse-width request. The current default calibration uses a 1,500 us neutral pulse, configured limits of 700–2,300 us, a forward direction sign of 1, and a software speed limit of 65%.

With that default mapping, host tests check example pulse values:

| Requested speed | Expected pulse in default forward mapping |
|---:|---:|
| 0% | 1,500 us |
| 2% | 1,516 us |
| 4% | 1,532 us |

These are calculated software command values, not measured RPM or physical speed.

### 7.3 Motor driver

`motor_driver.c` provides movement-level operations so navigation does not need to calculate servo pulses directly. The operations include initialization, stopping both motors, forward movement with separate left and right speed values, right turn, left turn, and reverse movement.

The left and right channels are controlled independently. Turning requires different commands for the two sides. Stopping requests the configured neutral pulse on both channels.

A neutral command is the software's stop request. It is not a physical power disconnect; actual servo behaviour must be verified on hardware.

## 8. HC-SR04 ultrasonic sensor

### 8.1 Measurement principle

The HC-SR04 has Trigger and Echo signals.

1. The controller issues a trigger pulse.
2. The sensor emits an ultrasonic burst.
3. The Echo output indicates the time associated with the reflected signal.
4. The driver measures the Echo pulse duration.
5. The driver converts that duration into a distance result.

Navigation uses the sensor driver interface rather than directly manipulating sensor pins.

### 8.2 Sensor driver operation

`hc_sr04_driver.c` handles starting a measurement, polling for completion, obtaining the result, and detecting invalid or timed-out measurements.

A conversion is limited by `MB_SENSOR_CONVERSION_TIMEOUT_MS`, configured to 40 ms. This prevents the driver from waiting indefinitely for a measurement that does not complete.

### 8.3 Retry handling

The obstacle-measurement path allows up to three attempts when a reading is invalid, according to `MB_SENSOR_RETRIES`.

If the attempts do not produce a valid reading, the measurement result is invalid. The navigation layer must not interpret that result as a clear path.

The host tests verify that three invalid attempts are made and that a conversion timeout is reported at the configured timeout.

### 8.4 Electrical requirement for later integration

The HC-SR04 Echo output is a 5 V signal. It must be level-shifted or otherwise safely interfaced before reaching an FPGA input. It must not be connected directly to an FPGA I/O pin without an interface designed to meet the FPGA's input limits.

## 9. Obstacle detection

`obstacle_detection.c` converts the sensor result into a path classification.

| Result | Meaning | Navigation response |
|---|---|---|
| `MB_PATH_CLEAR` | Valid measured distance is greater than 20 cm | Continue forward |
| `MB_PATH_BLOCKED` | Valid measured distance is 20 cm or less | Start obstacle recovery |
| `MB_PATH_INVALID` | A valid measurement could not be obtained | Initiate safety stop |

A failed measurement is deliberately different from a clear measurement. The robot must not assume that the path is safe when the sensor has failed.

The `check_path()` helper in `navigation.c` updates status:

- For `MB_PATH_INVALID`, it sets `st.sensor_fault` and returns the invalid result.
- For a valid result, it clears `st.sensor_fault` and records the latest valid distance in `st.last_distance_cm`.

## 10. Navigation state machine

### 10.1 Purpose

`navigation.c` implements the robot's movement state machine. The state machine makes the current operating mode explicit and controls which operations are performed on each navigation update.

The status structure includes information such as current state, whether the robot has started, selected speed, left and right speed percentages, last valid distance, sensor fault, stop latch, and recovery-cycle count.

### 10.2 States

The firmware defines states including:

| State | Purpose |
|---|---|
| `MB_STATE_INIT` | Initial state |
| `MB_STATE_WAIT_START` | Stopped and waiting for START |
| `MB_STATE_FORWARD` | Forward command active and path monitored |
| `MB_STATE_STOPPING` | Movement stopped before recovery |
| `MB_STATE_TURN_RIGHT` | Right-turn recovery action |
| `MB_STATE_TURN_LEFT` | Left-turn recovery action |
| `MB_STATE_REVERSE` | Reverse recovery action |
| `MB_STATE_RECHECK` | Path checked after a recovery action |
| `MB_STATE_SAFE_STOP` | Latched safety-stop state |
| `MB_STATE_SENSOR_FAULT` | Sensor-fault state defined in the types |

The current invalid-sensor safety path uses `MB_STATE_SAFE_STOP`; the separate `MB_STATE_SENSOR_FAULT` state is defined but is not used by this path.

### 10.3 Initialization and waiting

`navigation_init()` selects the scenario, selects SLOW as the initial speed, initializes speed values and recovery count, clears the started, sensor-fault and stop-latch flags, initializes the motor and sensor drivers, stops the motors, and enters `MB_STATE_WAIT_START`.

The robot therefore does not start moving automatically at initialization.

### 10.4 Starting forward movement

When `navigation_start()` is called and the stop latch is not active:

1. The `started` flag is set.
2. Current left and right ramp values are set to zero.
3. Target values are taken from the selected wheel-speed settings.
4. The ramp timer is initialized.
5. The next sensor check is scheduled.
6. Both motors are initially commanded to neutral through the forward-zero request.
7. Navigation enters `MB_STATE_FORWARD`.

The speed ramp then increases the requested wheel speeds gradually.

### 10.5 Periodic path monitoring

While in `MB_STATE_FORWARD`, each navigation update checks whether the next sensor deadline has been reached. When a check is due, the next check is scheduled using the configured sensor interval and `check_path()` is called.

- If the path is clear, forward operation continues.
- If the path is blocked, the motors are stopped and the recovery sequence begins.
- If the result is invalid, `safe_stop()` is called and navigation returns without continuing movement.

### 10.6 Recovery state operation

The state machine uses elapsed-time checks and recovery phases to progress through turning, reversing, and rechecking. The configured durations bound these actions. The selected scenario determines which sequence is attempted.

## 11. Obstacle-recovery scenarios

The firmware supports three scenarios. They use the same drivers and navigation framework but apply different recovery strategies.

### 11.1 Scenario 1 – Basic right-turn attempt

The intended sequence is:

1. Move forward.
2. Detect a blocked path.
3. Stop the motors.
4. Turn right for the configured duration.
5. Recheck the path.
6. If clear, resume forward movement.
7. If the path remains blocked, enter a safe-stop condition.

### 11.2 Scenario 2 – Right turn followed by left turn

The intended sequence is:

1. Move forward.
2. Detect a blocked path and stop.
3. Attempt a right turn.
4. Recheck the path.
5. If still blocked, attempt a left turn.
6. Recheck the path.
7. Resume if the path is clear; otherwise enter safe stop.

### 11.3 Scenario 3 – Recovery including reverse

The recovery scenario adds a reverse action and a bounded retry count:

1. Move forward and detect an obstacle.
2. Stop.
3. Attempt a right turn and recheck.
4. If still blocked, attempt a left turn and recheck.
5. If still blocked, reverse for a bounded duration and recheck.
6. Resume forward movement if the path becomes clear.
7. If the path remains blocked, repeat within the configured maximum of two recovery cycles.
8. If recovery is exhausted, enter `MB_STATE_SAFE_STOP`.

The configured right- and left-turn durations are 500 ms each. The maximum reverse duration is 300 ms. The maximum recovery-cycle setting is two.

These timings and sequences are firmware settings; physical navigation performance still requires testing with the actual robot.

## 12. Speed selection and gradual ramping

### 12.1 Speed modes

| Mode | Target percentage |
|---|---:|
| SLOW | 25% |
| MEDIUM | 45% |
| FAST | 65% |

The configured turn and reverse speed percentages are 35% and 30%. The wheel-speed interface also limits requested values to 65%.

### 12.2 Why ramping is used

Instead of immediately changing from zero to the selected target, navigation uses a gradual ramp. The configured ramp interval is 50 ms and the maximum increment or decrement per update is 2 percentage points.

For example, a start towards the SLOW target progresses as follows:

```text
0% -> 2% -> 4% -> 6% -> ... -> 24% -> 25%
```

The final step may be smaller than 2 percentage points so that the target is reached exactly.

### 12.3 Changing speed while moving

When a SPEED command is received while the robot is running, the selected mode and target wheel speeds are updated. The current speed is not immediately replaced by the new target. Subsequent ramp updates move the current software speed values towards the new target in configured increments.

The host test checks a change from 10% towards the FAST target of 65%, including the gradual 2-percentage-point progression.

The percentages are control-command percentages, not measured motor RPM.

## 13. UART command interface

`uart_command.c` handles incoming text commands and invokes the corresponding navigation operations.

| Command | Intended action |
|---|---|
| `START` | Request forward movement |
| `STOP` | Stop both motors and activate the configured stop latch |
| `STATUS` | Request/report current robot status through the command interface |
| `SPEED SLOW` | Select the 25% target |
| `SPEED MEDIUM` | Select the 45% target |
| `SPEED FAST` | Select the 65% target |
| `RESET` | Clear the navigation latch and sensor fault and return to WAIT_START |

Example command lines:

```text
START
STOP
STATUS
SPEED SLOW
SPEED MEDIUM
SPEED FAST
RESET
```

### 13.1 START

START requests the navigation system to begin forward movement. If the stop latch is active, `navigation_start()` rejects the request and leaves the robot stopped.

### 13.2 STOP

STOP commands both motors to neutral, clears the `started` flag, and enters `MB_STATE_SAFE_STOP`. With `MB_STOP_LATCHED_DEFAULT` enabled, it also activates the stop latch.

### 13.3 STATUS

STATUS requests the current robot status using the UART command/status interface. The status data is maintained by the navigation module.

### 13.4 SPEED

A SPEED command selects the requested target mode. While stopped, the selected target is reflected in stored speed settings. While moving, the ramp mechanism transitions the current speed towards the new target.

### 13.5 RESET

RESET is a navigation-control reset, not a reset of processor memory or project files. It:

- Commands both motors to stop.
- Clears the `started` flag.
- Clears `stop_latched`.
- Clears `sensor_fault`.
- Resets the recovery-cycle count.
- Sets the current ramp values to zero.
- Enters `MB_STATE_WAIT_START`.

It does not automatically start the robot. A new START command is required.

The current implementation preserves `last_distance_cm` and `selected_speed` during navigation reset. It does not erase source files, test results, or other project data.

## 14. Safety behaviour

### 14.1 Neutral motor command

The motor stop operation requests the configured neutral pulse (1,500 us) on both servo channels. The host tests verify the mock output values.

### 14.2 Stop latch

The stop latch records that a stop condition requiring operator acknowledgement has occurred. The configured `MB_STOP_LATCHED_DEFAULT` value is 1, so the latch is enabled.

When the latch is active, a START request is rejected. RESET is required before START can be accepted again.

### 14.3 Sensor-failure safe stop

1. Obstacle measurement exhausts its three invalid-reading attempts.
2. `check_path()` sets `st.sensor_fault`.
3. Navigation recognizes `MB_PATH_INVALID`.
4. `safe_stop()` commands the motors to stop.
5. `safe_stop()` clears `st.started`.
6. With stop-latching enabled, `safe_stop()` sets `st.stop_latched`.
7. Navigation enters `MB_STATE_SAFE_STOP`.
8. START is rejected while the latch remains active.
9. RESET clears the latch and sensor fault and returns to WAIT_START.
10. A new START is required.

The host test added for this behaviour verifies the fault flag, latch, neutral servo outputs, rejection of START before RESET, and acceptance of START after RESET.

### 14.4 Safety limitation

The software stop latch is not a substitute for a physical emergency-stop circuit or a hardware motor-power cutoff. A real robot needs an appropriate physical means of disabling motor power. Electrical design and physical fail-safe behaviour must be validated before operation.

## 15. Host-side test infrastructure

### 15.1 Purpose

The host test environment allows the C firmware to be compiled and exercised without the FPGA. It uses GCC and the mock implementation in `platform_host.c`.

Tests call firmware functions, inject simulated sensor and UART inputs, advance simulated time, and inspect resulting navigation status and servo pulse requests.

### 15.2 Test helpers

| Helper | Purpose |
|---|---|
| `platform_host_set_distance(cm, valid)` | Supplies a simulated distance and validity flag |
| `platform_host_set_sensor_delay(delay_ms)` | Configures a simulated sensor conversion delay |
| `platform_host_set_sensor_busy(busy)` | Simulates a busy sensor condition |
| `platform_host_feed_uart(text)` | Supplies command text to the UART mock |
| `platform_host_servo_pulse(wheel)` | Reads the mock pulse request for a wheel |
| `platform_host_advance_ms(ms)` | Advances simulated time |
| `platform_host_reset()` | Resets host mock state |
| `platform_host_sensor_start_count()` | Returns the number of sensor starts |
| `tick_ms(ms)` | Repeatedly calls navigation update and platform idle to advance a test by milliseconds |
| `reset_all()` | Initializes host platform, UART command handling, and navigation for a test |

These helpers make tests repeatable and allow timing and fault conditions to be exercised without physical hardware.

### 15.3 Compiler flags

The host build uses:

```text
-std=c11 -Wall -Wextra -Werror -pedantic -O2
-DMB_HOST_BUILD=1
```

- `-std=c11`: compile as C11.
- `-Wall -Wextra`: enable common warning groups.
- `-Werror`: treat warnings as errors.
- `-pedantic`: request strict standard-conformance diagnostics.
- `-O2`: enable compiler optimization.
- `-DMB_HOST_BUILD=1`: explicitly select the host platform build.

## 16. Phase 1 test coverage and observed result

### 16.1 Build and test command

The complete test suite was run from the workspace using:

```bash
make -C software clean && make -C software test
```

The command removes previous host executables, compiles the host test program, and runs it.

### 16.2 Observed result

The terminal reported:

```text
MicroBot host tests: 187 passed, 0 failed
```

**187 passed, 0 failed.**

This is the latest observed result after the sensor-failure safety test was added. The number is the total number of passing test assertions reported by the test harness, not a claim that there are 187 separate test functions.

### 16.3 Areas covered

The test suite includes checks for:

- Servo and motor output behaviour, including neutral output and turning direction.
- Sensor clear, blocked, and invalid classifications.
- Three invalid sensor measurement attempts.
- Sensor conversion timeout.
- Navigation startup and obstacle detection.
- UART command behaviour.
- SLOW, MEDIUM, and FAST selection.
- Gradual speed ramping.
- Speed changes while moving.
- STOP latch and RESET behaviour.
- Sensor-failure safety stop and RESET recovery.
- Scenario 1 right-turn and safe-stop behaviour.
- Scenario 2 right- and left-turn behaviour.
- Scenario 3 reverse recovery and recovery exhaustion.
- Reverse timing and early-clear handling.

### 16.4 Sensor-failure safety test

The added `test_sensor_failure_requires_reset()` exercises the navigation-level failure path:

1. Initialize the host platform and navigation.
2. Issue START and verify the FORWARD state.
3. Configure the sensor mock to return invalid readings.
4. Advance simulated time until the scheduled sensor check occurs.
5. Verify `MB_STATE_SAFE_STOP`.
6. Verify `started` is false.
7. Verify `sensor_fault` and `stop_latched` are true.
8. Verify both servo mock outputs equal the configured neutral pulse.
9. Call START without RESET and verify the robot remains stopped.
10. Call RESET and verify the fault and latch clear and the state becomes WAIT_START.
11. Call START again and verify FORWARD is accepted.

### 16.5 Meaning and limitations of the result

The result proves that the current host test build compiled with the stated GCC flags and that all assertions executed by the host test program passed.

It does not establish a successful Microwatt cross-compilation, a working Microwatt backend, correct Wishbone addresses or transactions, correct future VHDL peripheral operation, FPGA synthesis or timing closure, electrical compatibility, physical servo calibration, or successful physical obstacle avoidance.

Those require later phases and actual hardware validation.

## 17. Build and run the host tests

Run these commands from the firmware software directory.

### Step 1 – Enter the software directory

```bash
cd ~/aparna/microwatt-microbot/microbot_c_firmware_corrected/software
```

### Step 2 – Remove previous generated executables

```bash
make clean
```

This removes generated host test and scenario executables. It does not remove C source files, headers, or documentation.

### Step 3 – Compile and run the host tests

```bash
make test
```

The expected summary for the current test suite is:

```text
MicroBot host tests: 187 passed, 0 failed
```

### Step 4 – Build host scenario programs

```bash
make scenarios
```

This builds the configured host scenario executables. These are host programs, not Microwatt firmware images or FPGA bitstreams.

### Step 5 – Clean generated executables when required

```bash
make clean
```

## 18. Work remaining after Phase 1

The following tasks are not claimed as completed by Phase 1.

### 18.1 VHDL peripheral development

Implement and test the required hardware peripherals, including:

- Independent servo PWM generation.
- HC-SR04 Trigger and Echo handling.
- A confirmed timer/timebase.
- UART hardware interface.
- Wishbone register access and response logic.

### 18.2 Wishbone register-map definition

Define the real register addresses, offsets, widths, access behaviour, reset values, and status fields for each peripheral.

The C target backend and VHDL peripherals must use the same verified register definitions. Addresses must be derived from the actual Microwatt integration and peripheral design, not guessed.

### 18.3 Microwatt platform backend

Complete `platform_microwatt.c` after the real peripheral interfaces and register map have been established.

### 18.4 Target toolchain and firmware image

Inspect the actual Microwatt checkout and confirm:

- Supported cross-compiler and compiler options.
- CPU and memory configuration.
- Startup code and reset entry.
- Linker script and memory layout.
- Endianness and data representation.
- Firmware loading and execution procedure.

A host GCC executable is not a Microwatt firmware image.

### 18.5 FPGA and physical validation

After hardware integration:

- Verify FPGA clock and reset connections.
- Confirm actual pin constraints.
- Check servo PWM period and pulse widths on hardware.
- Calibrate each physical servo's neutral point and direction.
- Verify external servo power and common ground.
- Level-shift or protect the HC-SR04 Echo signal.
- Verify sensor timing and timeout behaviour on hardware.
- Test the physical emergency-stop and motor-power cutoff.
- Run controlled robot movement and obstacle-recovery tests.

## 19. Phase 1 completion statement

Phase 1 established the C firmware foundation and host-side test environment for MicroBot.

The firmware includes the application loop, platform abstraction, motor and servo drivers, ultrasonic sensor driver, obstacle classification, navigation state machine, three recovery scenarios, UART commands, speed selection, gradual speed ramping, and latched safety-stop handling.

The latest recorded host test result is **187 passed, 0 failed**.

The Phase 1 host-validation work is complete against the current test suite. The target Microwatt backend, VHDL peripherals, Wishbone integration, FPGA implementation, and physical robot validation remain future work.

## 20. Next phase

**Phase 2 – Microwatt source and hardware-interface inspection**

The next phase will inspect the actual Microwatt source tree and platform implementation. The work will identify the existing processor bus architecture, timer, UART, GPIO, FPGA top-level connections, and relevant constraints.

That inspection will provide the evidence needed to define the MicroBot Wishbone peripheral interface and register map. No target register addresses should be added to the C firmware until the hardware design confirms them.

---

## Document record

- Project: MicroBot based on OpenPOWER Microwatt
- Phase: 1 – C Firmware Development and Host Validation
- FPGA target: Digilent Arty A7-100T
- Host compiler: GCC
- Host build selection: `MB_HOST_BUILD=1`
- Latest observed host test result: 187 passed, 0 failed
- Microwatt target build: Pending
- VHDL/Wishbone integration: Pending
- FPGA and physical robot validation: Pending
