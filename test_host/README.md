# Host test tier

Native unit tests for the portable libraries (`kinematics`, `pid`, `odom_integrator`).
Runs on the host with no MCU, board, or ROS install — Tier A of the emulation-first
strategy in `docs/STM32CUBE_PORTING_PLAN.md`.

## Prerequisites
- `g++` (C++17) and GNU `make`. `doctest.h` is vendored in this directory (v2.4.11, MIT) — no download needed.

## Run
```bash
cd test_host
make test
```
Expected: `[doctest] Status: SUCCESS!`, exit code 0.

## Layout
- `shim/Arduino.h` — minimal Arduino API shim (`PI`, `constrain`; `fabs` via `<math.h>`) for host builds.
- `doctest.h` — vendored single-header framework (v2.4.11, MIT).
- `test_*.cpp` — one TU per library under test; `test_main.cpp` owns `main()`.
