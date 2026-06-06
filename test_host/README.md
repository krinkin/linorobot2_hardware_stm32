# Host test tier

Native unit tests for the portable libraries **and the pure math helpers** of the STM32
port -- `kinematics`, `pid`, `odom_integrator`, plus `encoder_math`, `pwm_timing`,
`imu_math`, `motor_dispatch`, and `control_core` (all in `firmware_stm32/Inc`). Runs on the
host with no MCU, board, or ROS install -- Tier A of the emulation-first strategy in
`docs/STM32CUBE_PORTING_PLAN.md`.

## Prerequisites
- `g++` (C++17) and GNU `make`. `doctest.h` is vendored in this directory (v2.4.11, MIT) -- no download needed.

## Run
```bash
make test-host        # from the repo root -- what CI and `make test-all` invoke
# or, in this directory:
cd test_host && make test
```
Expected: `[doctest] Status: SUCCESS!`, exit code 0.

## Layout
- `shim/Arduino.h` -- minimal Arduino API shim (`PI`, `constrain`; `fabs` via `<math.h>`) for host builds.
- `doctest.h` -- vendored single-header framework (v2.4.11, MIT).
- `test_*.cpp` -- one TU per library under test; `test_main.cpp` owns `main()`.
