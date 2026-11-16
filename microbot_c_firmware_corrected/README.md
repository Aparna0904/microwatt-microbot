# MicroBot – OpenPOWER Microwatt-Based Autonomous Robot

## 1. Project Overview

MicroBot is an autonomous obstacle-detection and avoidance robot designed around the OpenPOWER Microwatt processor.

The project aims to implement the robot control system using C firmware running on Microwatt, with custom VHDL peripherals connected through the Wishbone bus.

The robot uses an ultrasonic sensor to detect obstacles and two independently controlled continuous-rotation servo motors for movement.

The firmware is designed to support forward movement, obstacle detection, automatic recovery, speed selection, UART commands and safety-stop operation.

**Target FPGA:** Digilent Arty A7-100T
**Processor:** OpenPOWER Microwatt

## 2. Hardware Components

| Component | Purpose |
|---|---|
| OpenPOWER Microwatt | Main processor |
| Digilent Arty A7-100T | FPGA development board |
| Two FS5106R servos | Independent left and right wheel control |
| HC-SR04 ultrasonic sensor | Forward obstacle detection |
| UART | Communication between the operator and robot |
| Wishbone peripherals | Interface between processor and custom peripherals |

The HC-SR04 sensor is mounted facing forward. It measures the distance to objects in front of the robot.

The two servo motors are controlled independently to support forward movement, turning, reverse movement and stopping.

**Electrical safety:** The HC-SR04 Echo signal is 5 V and must be level-shifted or protected before connecting it to an FPGA input. The servos require an appropriate external power supply and a common ground with the FPGA.

## 3. Software Architecture

The firmware is organized into separate modules so that each component has a clearly defined responsibility.

| Module | Responsibility |
|---|---|
| Main application | Initializes the platform and coordinates firmware execution |
| Platform abstraction | Separates host simulation from hardware-specific implementation |
| Servo driver | Converts speed and direction requests into servo pulse-width commands |
| Motor driver | Controls forward, reverse, turning and stopping operations |
| HC-SR04 driver | Handles ultrasonic sensor measurements and timeouts |
| Obstacle detection | Classifies the path as clear, blocked or invalid |
| Navigation | Controls robot movement and obstacle-recovery sequences |
| UART command handler | Processes operator commands |
| Scenario module | Selects the navigation strategy |

### 3.1 Main Execution Sequence

The firmware follows this sequence:

1. Initialize the platform.
2. Initialize UART command handling.
3. Initialize navigation and select the operating scenario.
4. Wait for the START command.
5. Move forward using gradual speed ramping.
6. Check the ultrasonic sensor at configured intervals.
7. Continue forward if the path is clear.
8. Start obstacle recovery if the path is blocked.
9. Enter a latched safety stop if sensor readings remain invalid.

The main application continuously polls UART commands, updates navigation and executes the platform idle operation.

## 4. Robot Operating Behaviour

### 4.1 Forward Movement

After receiving START, the robot enters the forward state.

Both servo motors initially receive neutral commands. Their requested speeds then increase gradually until the selected target speed is reached.

### 4.2 Obstacle Detection

The ultrasonic sensor checks the forward path every 100 ms.

The configured obstacle threshold is 20 cm.

- Distance greater than 20 cm: path is clear.
- Distance at or below 20 cm: obstacle detected.
- Invalid measurement: sensor fault handling is initiated.

### 4.3 Obstacle Recovery Scenarios

Three navigation scenarios are implemented.

**Scenario 1 – Basic Right Turn**

The robot stops when an obstacle is detected, attempts a right turn and checks the path again. If the path remains blocked, it enters a safe-stop condition.

**Scenario 2 – Right and Left Turns**

The robot attempts a right turn first. If the path remains blocked, it attempts a left turn and checks again. If neither direction clears the path, it stops safely.

**Scenario 3 – Reverse Recovery**

The robot attempts right and left turns, followed by a limited reverse movement and another path check. Recovery can repeat up to the configured maximum of two cycles. If recovery is unsuccessful, the robot enters SAFE_STOP.

All three scenarios use the same navigation framework and device drivers.

## 5. Speed Control

The firmware supports three selectable speed modes.

| Mode | Target speed |
|---|---:|
| SLOW | 25% |
| MEDIUM | 45% |
| FAST | 65% |

Additional configured values:

- Turning speed: 35%.
- Reverse speed: 30%.

The firmware uses gradual speed ramping instead of immediately applying the selected target.

The configured ramp interval is 50 ms, with a speed change of up to 2 percentage points per update.

When the speed mode is changed during movement, the current speed gradually moves towards the new target.

These percentages represent software control requests, not measured motor RPM.

## 6. UART Command Interface

The robot supports the following text commands:

| Command | Operation |
|---|---|
| START | Begin forward movement |
| STOP | Stop both motors and activate the stop latch |
| STATUS | Request current robot status |
| SPEED SLOW | Select 25% target speed |
| SPEED MEDIUM | Select 45% target speed |
| SPEED FAST | Select 65% target speed |
| RESET | Clear the navigation safety latch and return to WAIT_START |

The robot does not automatically restart after STOP or a latched sensor failure.

RESET returns navigation to WAIT_START. A new START command is required.

## 7. Safety Mechanisms

The firmware includes the following safety behaviour:

- Both servo outputs are commanded to the configured neutral pulse of 1500 us when stopping.
- The STOP latch prevents unintended restarting.
- Invalid sensor readings are not treated as a clear path.
- The ultrasonic measurement has a 40 ms conversion timeout.
- Up to three invalid measurement attempts are permitted.
- A sensor failure activates the stop latch when the configured latch option is enabled.
- RESET clears the navigation fault and latch but does not automatically restart the robot.

The software stop mechanism is not a replacement for a physical emergency-stop circuit or motor power cutoff.

## 8. Project Directory

The main project is organized into the following structure:

```text
microwatt-microbot/
|
+-- microbot_c_firmware_corrected/
    |
    +-- README.md
    +-- PHASE1_C_FIRMWARE_HOST_VALIDATION.md
    |
    +-- software/
        |
        +-- include/
        |   +-- Configuration and interface headers
        |
        +-- src/
        |   +-- Main application
        |   +-- Platform implementations
        |   +-- Motor and servo drivers
        |   +-- Ultrasonic sensor driver
        |   +-- Obstacle detection
        |   +-- Navigation
        |   +-- UART command handling
        |   +-- Scenario selection
        |
        +-- tests/
        |   +-- test_main.c
        |
        +-- Makefile
        +-- Supporting documentation
```

## 9. Phase 1 – C Firmware Development and Host Validation

Phase 1 established the C firmware foundation and host-side testing environment.

The completed software work includes:

- Application initialization and main execution loop.
- Hardware abstraction layer.
- Host-side simulated platform.
- Servo and motor drivers.
- HC-SR04 sensor driver.
- Obstacle detection logic.
- Navigation state machine.
- Three obstacle-recovery scenarios.
- UART command handling.
- Speed selection and gradual ramping.
- STOP latch and sensor-failure safety handling.
- Host-side test infrastructure.

### 9.1 Host Test Result

The latest complete host test execution reported:

**187 passed, 0 failed.**

The build uses GCC with C11, strict warning checks and the explicit host platform selection.

To run the tests from the project root:

```bash
make -C microbot_c_firmware_corrected/software clean
make -C microbot_c_firmware_corrected/software test
```

To build the host scenario programs:

```bash
make -C microbot_c_firmware_corrected/software scenarios
```

These are host executables, not FPGA bitstreams or Microwatt firmware images.

### 9.2 Detailed Phase 1 Documentation

For the complete technical explanation of the firmware architecture, module responsibilities, sensor operation, navigation state machine, recovery sequences, UART commands, safety logic, test infrastructure and validation results, see:

**[Phase 1 – C Firmware Development and Host Validation](PHASE1_C_FIRMWARE_HOST_VALIDATION.md)**

## 10. Current Implementation Status

### Completed in Phase 1

- C firmware development.
- Host platform simulation.
- Navigation and recovery logic.
- UART command handling.
- Software safety mechanisms.
- Host test suite: 187 passed, 0 failed.

### Pending Hardware Integration

- VHDL peripheral implementation.
- Wishbone register-map definition.
- Microwatt target platform backend.
- Timer and UART hardware binding.
- Target cross-compilation and firmware image generation.
- FPGA synthesis and implementation.
- Physical motor and sensor testing.

The host-side test result must not be interpreted as proof of FPGA or physical robot operation.

## 11. Future Development – Phase 2

### Microwatt Source and Hardware-Interface Inspection

The next phase will inspect the actual Microwatt source code and platform implementation.

The work will identify:

1. Existing processor bus architecture.
2. Wishbone integration and address mapping.
3. Available timer implementation.
4. UART interface.
5. GPIO and FPGA top-level connections.
6. Clock, reset and pin constraints.

The findings will be used to define the MicroBot hardware interface and the actual Wishbone register map.

No hardware register addresses will be guessed or introduced before the interface is confirmed.

---

**Project:** MicroBot
**Processor:** OpenPOWER Microwatt
**FPGA:** Digilent Arty A7-100T
**Current milestone:** Phase 1 – C Firmware Host Validation
**Latest observed result:** 187 passed, 0 failed

## Project Documentation

- [Complete MicroBot Implementation Plan (PDF)](../docs/MicroBot_Implementation_Plan.pdf)
- [Phase 1: C Firmware Host Validation](PHASE1_C_FIRMWARE_HOST_VALIDATION.md)
