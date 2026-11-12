# Build instructions

## Host software validation
Requires a C11 compiler such as GCC or Clang.
```sh
cd microbot/software
make clean
make test
make scenarios
```

## Target firmware
A real Microwatt target build is intentionally blocked until:
- the VHDL Wishbone register map is finalized,
- timer and UART hardware interfaces are identified,
- `platform_microwatt.c` is implemented,
- the existing Microwatt firmware startup/linker/toolchain conventions are inspected.

Do not define `MB_MICROWATT_REGISTER_MAP_FINALIZED` to bypass the guard. It is not an implementation.
