# UART commands

Commands are ASCII lines terminated by CR or LF:
- `START`
- `STOP`
- `STATUS`
- `SPEED SLOW`
- `SPEED MEDIUM`
- `SPEED FAST`
- `RESET`
- `DEBUG DETAILED`
- `DEBUG EVENT`

Power-on always enters WAIT_START. STOP immediately commands both channels to calibrated neutral and enters SAFE_STOP. RESET returns to WAIT_START; START is then required again.

Detailed mode currently exposes explicit STATUS and event messages. Periodic telemetry can be added after the UART throughput/baud rate is finalized.
