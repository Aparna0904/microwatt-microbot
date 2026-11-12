# Test plan and limits

Host tests exercise servo pulse mapping, motor direction combinations, distance classification, start/stop/reset command flow and a basic recovery-success path.

Additional verification required before hardware deployment:
- Exhaustive state-transition tests for both recovery cycles.
- Sensor asynchronous completion and timeout tests against the real VHDL timing model.
- UART parser overflow/noise tests.
- STOP priority under each navigation state.
- Servo calibration and safe pulse bounds on real hardware.
- GHDL testbenches for each VHDL peripheral.
- Full Microwatt + Wishbone + C firmware system simulation.

Host tests do not prove electrical behaviour or FPGA integration.
