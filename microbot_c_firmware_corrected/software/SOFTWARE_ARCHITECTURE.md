# Software architecture

`main.c` initializes the platform, UART command parser and selected navigation scenario.
Navigation is a cooperative state machine; it does not use long blocking delays. UART polling occurs before each navigation tick so STOP can be serviced promptly.

Layering:
- `platform.h`: hardware abstraction contract.
- `platform_host.c`: deterministic test backend.
- `platform_microwatt.c`: fail-closed integration placeholder.
- Drivers: HC-SR04, servo, motor.
- Navigation: distance classification and state machine.
- UART: line parser and command dispatch.
- Scenario selection: compile-time independent program selection.

All three scenarios share identical drivers and HAL. Scenario 1 is basic right-turn-only then safe stop. Scenario 2 tries right then left and safe-stops if both fail. Scenario 3 adds reverse/recheck and at most two full recovery cycles.
