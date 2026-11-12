# Hardware interface contract required before Microwatt binding

The C code expects the target HAL to provide:

1. Monotonic millisecond timer (`platform_millis`) and idle/yield (`platform_idle`).
2. HC-SR04 start command and non-blocking completion result with `valid`, `timeout`, distance in cm.
3. Two servo pulse-width registers accepting pulse width in microseconds or documented timer ticks.
4. UART RX byte polling and TX string output.
5. Reset and clock behaviour documented by the SoC integration.

The following must be agreed with VHDL before target implementation:
- Wishbone address map, data width, byte select behaviour and access latency.
- Sensor command/status/data register bit definitions.
- Servo channel register widths, units, update/enable semantics and PWM period.
- Timer tick frequency and rollover behaviour.
- UART baud rate, FIFO status and register semantics.
- Interrupt usage (this C baseline uses polling and does not require interrupts).

HC-SR04 Echo is a 5 V signal on common modules: use a suitable divider/level shifter before FPGA input.
