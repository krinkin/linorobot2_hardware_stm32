# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A **GUI-free native STM32Cube/HAL port** of the linorobot2 low-level micro-ROS base controller,
living in `firmware_stm32/`. It is the microcontroller-side firmware: it subscribes to `/cmd_vel`,
runs per-wheel PID over encoder feedback to drive the motors, reads an IMU, and publishes
`/odom/unfiltered` + `/imu/data_raw` back to ROS 2 over micro-ROS. It is built directly on ST HAL +
FreeRTOS + CMSIS with **no STM32CubeMX, no .ioc, no GUI** -- everything is hand-written from pinned
OSS submodules and a hand-written Makefile. First board: NUCLEO-F446RE (Cortex-M4F); designed to be
universal across any FPU STM32 (config-driven TIM/I2C/pin descriptor tables, so a second board is a
new config header with zero driver edits).

The matching ROS 2 nodes live in the separate [linorobot2](https://github.com/linorobot/linorobot2)
repo. This repo's default branch is `stm32` (repo: `krinkin/linorobot2_hardware_stm32`). It targets
the ROS 2 **Jazzy** distro -- "jazzy" in image tags / `/opt/ros/jazzy` / the `micro_ros_stm32cubemx_utils`
submodule branch is the *distro*, not a git branch. The upstream multi-board Arduino firmware is not
maintained here; only the platform-agnostic libraries the port reuses are kept under `firmware/lib/`.

## Build / flash / test

No host ROS install is needed (libmicroros is built by a self-contained Docker image; ros2 and the
micro-ROS agent run only inside the agent container). One-time: `git submodule update --init --recursive`.
There are two ways to run -- pick one (full guide in `docs/TESTING.md`):

**Docker only (recommended for a clean machine):** install nothing but Docker.
```bash
make docker-test-all     # build the dev image, run the WHOLE suite in it -> "ALL TIERS GREEN"
make docker-shell        # interactive shell in the dev image
make docker-<target>     # run any single target in the image, e.g. make docker-build-fw
```
The dev image (`docker/Dockerfile`) carries the ARM toolchain + Renode + socat; `make libmicroros`
and the agent tiers run as *sibling* containers via the bind-mounted host Docker socket (DooD).

**Native tools on the host:** distro `gcc-arm-none-eabi` (13.2.x) + binutils + newlib, GNU make,
docker, socat, and Renode 1.16.1 portable (`export RENODE=/path/to/renode`), then `make test-all`.

Tiers (each runs standalone, or via `make docker-<t>`):
- **Tier A** `make test-host` -- host doctest unit tests (kinematics/pid/odom + the pure math seams).
- **Tier B** `make libmicroros` (pinned Docker builder) then `make build-fw` -> `firmware_stm32/build/firmware_stm32.elf` (a clean link is `text=108896`).
- **Tier C (Renode, no board)** `make renode` (F2 boot + micro-ROS ping), `make control` (F4 encoder/PWM + odometry), `make imu` (F5 MPU6050 over HAL I2C against a Python mock).
- **Tier C+ (live micro-ROS agent)** `make agent-roundtrip` (F3 XRCE session), `make topics` (F6 full cmd_vel/odom/imu round-trip).

CI: `.github/workflows/stm32-f446re.yml` is the only workflow -- Tiers A/B + F2/F4/F5 on push and PR,
F6 on push. There is no PlatformIO/Arduino matrix CI. Phases are F0-F7 (F0-F6 done in emulation; F7 =
real hardware bring-up). Milestone git tags use `stm32cube-p<N>-<name>`.

## Architecture

`firmware_stm32/` is a hand-written HAL/FreeRTOS project. Two FreeRTOS tasks:
- **control_task** (`Src/control_loop.cpp`): the 50 Hz loop -- `kinematics.getRPM(cmd_vel)` ->
  per-motor `PID.compute(target, encoder.getRPM())` -> `motor.spin(pwm)`, then velocities ->
  odometry, plus the IMU read. Runs regardless of the agent; a 200 ms `/cmd_vel` deadman brakes the
  base. It exposes a plain-C snapshot surface (`control_set_cmd` / `control_get_odom` /
  `control_get_imu`) guarded by `taskENTER/EXIT_CRITICAL` -- no C++ type crosses the task boundary.
- **uros_task** (`Src/main.c`): the micro-ROS half -- rclc executor + a 4-state reconnect machine
  (WAITING_AGENT -> AGENT_AVAILABLE -> AGENT_CONNECTED -> AGENT_DISCONNECTED), recreating entities on
  agent loss. A publish-only 50 Hz timer reads the snapshot and publishes `/odom/unfiltered` +
  `/imu/data_raw` (wall-clock stamps from `rmw_uros_epoch_nanos`, REP-145 orientation).

Platform layer -- the ONLY place HAL/CMSIS symbols may appear:
- `Src/stm32_encoder.cpp`, `Src/stm32_motor.cpp`, `Src/stm32_i2c.cpp` -- TIM encoder mode, TIM PWM +
  GPIO direction, the HAL I2C adapter.
- `Src/lino_hal.cpp` -- timebase: `lino_millis()` = the FreeRTOS tick (no DWT; Renode does not model
  the DWT cycle counter), `lino_micros()` = a free-running 1 MHz TIM5.
- `Src/sys_freertos.c` -- newlib `__malloc_lock/unlock` (vTaskSuspendAll), a bounded `_sbrk`, the
  stack-overflow hook.
- `Src/microros_glue.c` -- the micro-ROS custom UART transport (open/close/write/read over HAL
  USART2); `Src/microros_atomic64.c` -- a 64-bit-atomics PRIMASK shim (rcl references `__atomic_*_8`,
  which arm-none-eabi cannot satisfy on Cortex-M; remove it and the link fails).

Reused unchanged from the shared libs: `firmware/lib/{kinematics,pid,odom_integrator}` (kinematics
and pid are compiled in) and `firmware/lib/imu/{imu,mag}_interface.h`. The IMU/motor drivers are the
port's OWN (`Inc/stm32_imu.h` = Mpu6050Imu + FakeIMU; `Inc/stm32_mag.h` = FakeMAG;
`Inc/motor_interface.h`), NOT the Arduino ones.

Host-testable pure seams (header-only, in `Inc/`, no HAL): `control_core.h` (the moveBase math),
`encoder_math.h`, `pwm_timing.h`, `imu_math.h` -- this is what Tier A covers.

Config: `firmware_stm32/config/{config.h, f446re_config.h}` -- `config.h` selects a board by
`USE_*_CONFIG` (build flag `-DUSE_F446RE_CONFIG`); `f446re_config.h` holds the robot geometry, PID
gains, and the TIM/I2C/pin descriptor tables. A second board = a new config header, no driver edits.

libmicroros is built once by the pinned `microros/micro_ros_static_library_builder` Docker image.
The ABI matches by construction: the builder reads the firmware's real CFLAGS via `make print_cflags`
and compiles the library with the same `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard` (hard-VFP,
no libstdc++); `colcon.meta` sets `RCUTILS_NO_64_ATOMIC=ON`.

For depth: `docs/STM32CUBE_PORTING_PLAN.md` (design spec, phases F0-F7), `docs/superpowers/plans/`
(per-phase as-built records), and `docs/TESTING.md` (tiers + the Renode emulation caveats: DWT not
modeled, TIMs free-run, `Mocks.DummyI2CSlave` returns 1 byte per read, socket terminals need
`emitConfigBytes=false`).

## Conventions

- **English-only, ASCII, no emoji** in all code, docs, commit messages, and any development artifact.
  (Conversation may be in another language; artifacts must not be.) The phase marker is ASCII `F0`-`F7`.
- **No upstream legacy.** This repo does not maintain the upstream linorobot2_hardware Arduino /
  PlatformIO apparatus -- remove inherited cruft rather than repairing it; keep only what the STM32
  port actually uses (the shared `firmware/lib` subset above).
- **Tag milestones** with `git tag` (push optional), convention `stm32cube-p<N>-<name>`.
- Keep HAL/CMSIS symbols confined to the `Src/stm32_*.cpp` + `lino_hal` layer; the rest of the port is
  platform-agnostic and host-testable. Put new pure logic behind a header seam in `Inc/` and cover it
  in `test_host/`.
- `firmware_stm32/vendor/` and `micro_ros_stm32cubemx_utils` are pinned submodules -- do not edit;
  bump by commit.
- CI must stay green and ASCII; the Renode smokes gate on exit codes (see `firmware_stm32/renode/`).
