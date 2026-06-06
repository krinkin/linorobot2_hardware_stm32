# Building & testing the STM32 native (GUI-free) port

This is your hands-on guide to **build everything and drive the tests yourself**. No
STM32CubeMX, no GUI -- everything is a `make` target you can run and reason about.

## 1. The mental model: three tiers

Testing is split into three independent tiers, each proving a different thing. You
run them bottom-up; a failure in a lower tier explains failures above it.

| Tier | What it proves | Needs | Speed |
|---|---|---|---|
| **A -- host unit tests** | the portable math (`kinematics`, `pid`, `odometry`) is correct | `g++`, `make` | seconds |
| **B -- firmware F0 link** | the real F446 firmware **links** micro-ROS (float-ABI + 64-bit atomics OK) | `arm-none-eabi-gcc`, `docker` | ~1 min (lib) + seconds |
| **C -- Renode boot (F2)** | the firmware **comes alive**: FreeRTOS scheduler ticks, `uros_task` runs, and it **transmits the micro-ROS ping** on USART2 | `renode`, `socat` | ~1 min |
| **C -- control loop (F4)** | the **encoder/PWM control loop** runs at 50 Hz and an **injected encoder count moves odometry** (CNT->getRPM->odom, frozen-timer load-bearing) | `renode` | ~1 min |
| **C -- IMU (F5)** | the firmware **reads an MPU6050 over the real HAL I2C path** (WHO_AM_I + config) against a Python mock slave | `renode` | ~1 min |
| **C+ -- agent round-trip (F3)** | a real `micro_ros_agent` establishes a **live XRCE session** with the emulated firmware (node `stm32_node`) | `renode`, `docker`, `socat` | ~1 min |
| **C+ -- topics (F6)** | the full base node over a live agent: `/cmd_vel` sub + `/odom/unfiltered` + `/imu/data_raw` pubs (frames + reconnect) | `renode`, `docker`, `socat` | ~1.5 min |

Why split this way: Tier A runs anywhere with zero embedded toolchain (fast TDD on the
math); Tier B is a pure *link* gate (the riskiest thing in the whole port -- see section 7);
Tier C is a *runtime* gate in emulation (no hardware needed).

## 2. Prerequisites

```bash
# one-time: pull the vendored sources (CMSIS / HAL / FreeRTOS / micro-ROS utils)
git submodule update --init --recursive
```

Then pick **one** of two setups:

**(a) Docker only -- recommended for a clean machine.** Install nothing but Docker; a
self-contained dev image (`docker/Dockerfile`) carries the ARM toolchain + Renode + socat,
and every tier runs inside it via the `docker-*` targets (see section 3). This is the least-surprise
path -- it's exactly what bites people who clone onto a box without Renode installed.

**(b) Native tools on the host:**
```bash
# toolchain (Debian/Ubuntu)
sudo apt-get install -y g++ make gcc-arm-none-eabi binutils-arm-none-eabi docker.io socat
# Renode 1.16.1 (portable build -- bundles its own runtime, no system mono needed):
curl -L https://github.com/renode/renode/releases/download/v1.16.1/renode-1.16.1.linux-portable.tar.gz \
  | tar -xz -C "$HOME"
export RENODE="$HOME/renode_1.16.1_portable/renode"   # the Tier-C smokes read $RENODE (or PATH)
```
No CubeMX, no myST account, no `.ioc`. Docker is used only to build `libmicroros.a`
(the prebuilt micro-ROS static library) and to run the micro-ROS agent (Tier C+); the
firmware itself is built by `arm-none-eabi-gcc` from a hand-written `Makefile`.

## 3. Quick start

**Docker only (setup a):**
```bash
make docker-test-all     # build the dev image, then run Tier A+B+C inside it -> "ALL TIERS GREEN"
make docker-shell        # interactive shell in the image
make docker-<target>     # run any single target in the image, e.g. make docker-build-fw
```
The dev image runs `make libmicroros` and the agent tiers as *sibling* containers over the
bind-mounted host Docker socket (Docker-out-of-Docker), so the only host dependency is Docker.

**Native tools (setup b):**
```bash
make help        # list the targets
make test-all    # Tier A + B + C, in order  ->  "ALL TIERS GREEN"
```
Or drive each tier:
```bash
make test-host   # Tier A
make libmicroros # build libmicroros.a (once; cached afterwards)
make build-fw    # Tier B (link firmware)  -> firmware_stm32/build/firmware_stm32.elf
make renode      # Tier C  (F2 boot smoke -- firmware transmits the ping)
make control     # Tier C  (F4 -- encoder/PWM control loop + injected-encoder->odom)
make imu         # Tier C  (F5 -- MPU6050 read over real HAL I2C via a Python mock slave)
make agent-roundtrip   # Tier C+ (F3 -- live micro_ros_agent <-> firmware XRCE session)
make topics      # Tier C+ (F6 -- full base node: cmd_vel + odom/unfiltered + imu/data_raw)
```

## 4. Tier A -- host unit tests (the math)

```bash
make test-host       # == make -C test_host clean && make -C test_host test
```
- **What runs:** `test_host/` compiles `kinematics`/`pid`/`odom_integrator` with plain
  `g++` against a tiny `Arduino.h` shim + the vendored `doctest` single-header framework.
- **Reading the result:** ends with `[doctest] Status: SUCCESS!` and `N passed | 0 failed`.
- **Add a test:** drop a `test_host/test_<thing>.cpp` (the `Makefile` auto-discovers
  `test_*.cpp` via `$(wildcard)`), `#include "doctest.h"` + the header under test, write
  `TEST_CASE("...") { CHECK(...); }`, then `make test-host`. The framework is doctest, so
  use `CHECK`, `REQUIRE`, `doctest::Approx(x)` (note: doctest has `.epsilon()`, **not**
  Catch2's `.margin()` -- use `std::fabs(x) < eps` for near-zero checks).
- See `test_host/README.md` for the layout.

## 5. Tier B -- firmware F0 link

```bash
make libmicroros     # build the micro-ROS static lib (pinned Docker image) -- once
make build-fw        # link firmware_stm32  ->  build/firmware_stm32.elf (+ .hex/.bin)
```
- **What `make libmicroros` does:** runs the pinned `micro_ros_static_library_builder`
  Docker image; it reads the firmware's exact compile flags via `make print_cflags` and
  compiles `libmicroros.a` with the *same* `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16
  -mfloat-abi=hard`. This is what guarantees the float-ABI match (no "VFP register
  arguments" wall). Output: `firmware_stm32/micro_ros_stm32cubemx_utils/.../libmicroros.a`.
- **What `make build-fw` does:** compiles the hand-written project (CMSIS startup +
  `system_stm32f4xx.c` + STM32 HAL + FreeRTOS + `Src/main.c`/`microros_glue.c` +
  the 64-bit-atomic shim `microros_atomic64.c`) and links it against `libmicroros.a`.
- **Reading the result:** the `arm-none-eabi-size` line (~ 49 KB Flash / 73.5 KB RAM)
  and a clean link. Confirm hard-float: `arm-none-eabi-readelf -A
  firmware_stm32/build/firmware_stm32.elf | grep Tag_ABI_VFP_args` -> `VFP registers`.
- **The non-obvious part -- the 64-bit atomic shim:** `rcl` uses `__atomic_*_8` and
  `arm-none-eabi` has no baremetal 64-bit atomics on Cortex-M4, so
  `Src/microros_atomic64.c` provides them (PRIMASK critical sections). Remove it and the
  link fails with `undefined reference to __atomic_load_8` -- that's the single most
  important gotcha of the whole port (see section 7).

## 6. Tier C -- Renode boot smoke (F2)

```bash
make renode          # boots build/firmware_stm32.elf in Renode, asserts no HardFault
# (set RENODE=/path/to/renode if it isn't on PATH)
```
- **What it does:** `firmware_stm32/renode/boot_smoke.sh` loads the ELF on Renode's
  generic `stm32f4.repl`, wires USART2 to a raw TCP socket, and reads it with `socat`.
  PASS iff the firmware **emits bytes** -- the `rmw_uros_ping_agent` GET_INFO frame. That
  is a strictly stronger gate than "no HardFault": it proves the scheduler started, the
  task was scheduled, and the HAL-UART transport ran. CI runs `renode/boot_smoke.robot`,
  which asserts the same liveness a different way -- `xTickCount != 0` and
  `uxCurrentNumberOfTasks != 0` (the scheduler actually ticked).
- **Reading the result:** `[OK] F2 PASS: scheduler + uros_task + UART transport alive`.
- **Why the strong check matters (a real bug it caught):** a `configASSERT` failure
  (`taskDISABLE_INTERRUPTS(); for(;;);`) is **not** a HardFault -- it's an interrupt-disabled
  spin. A "PC != HardFault" smoke *passes* such a startup hang. The ping/tick check does
  not. See section 7.5.
- **Emulator caveat:** the HAL timebase is the FreeRTOS tick (not DWT -- Renode doesn't
  model the DWT cycle counter), so HAL-UART timeouts fire correctly in emulation. Neither
  emulator is baud/timing-accurate, so wire framing and real-time jitter remain
  hardware-only checks.

## 6b. Tier C+ -- agent round-trip (F3)

```bash
make agent-roundtrip   # Renode raw socket <-> socat <-> micro_ros_agent (Docker)
```
- **What it does:** `firmware_stm32/renode/agent_bridge.sh` runs the firmware in Renode
  with USART2 on a **raw** TCP socket, and bridges a real `micro_ros_agent` to it. The
  firmware's wait-for-agent loop (`rmw_uros_ping_agent`) connects, then
  `rclc_node_init_default("stm32_node")` creates a participant. PASS iff the agent logs
  `session established` + `participant created`.
- **The bridge, and why it's shaped this way:** a Docker container **cannot open a host
  pty** across the devpts namespace, so `socat` (TCP<->pty) **and** the agent run inside one
  `--network host` container -- the pty is then local to that container. The script picks a
  free port (a just-killed Renode leaves the old one in `TIME_WAIT`; Renode's socket has no
  `SO_REUSEADDR`) and uses a passive `ss` listen check so `socat` is the socket terminal's
  sole client.
- **Reading the result:** `[OK] F3 PASS: live micro-ROS round-trip -- XRCE session + participant created`.
- Not in CI by default (needs Docker + socat on the runner; builds a ~500 MB socat-augmented
  agent image on first run). Run it locally / as an opt-in gate.

## 7. The two link gotchas (why Tier B is the risk)

The native micro-ROS-on-Cortex-M port has exactly two link-time walls; both are handled,
but know them so you can read a failure:
1. **Float-ABI / VFP** -- `... uses VFP register arguments, libmicroros.a(...) does not`.
   Avoided **by construction**: the Docker builder forwards `print_cflags`, so the lib is
   built hard-float. If you see it, `print_cflags` didn't emit `-mfloat-abi=hard` (check the
   `Makefile`'s `MCU`/`FLOAT-ABI`).
2. **64-bit atomics** -- `undefined reference to __atomic_load_8` (from `rcl` time/timer/
   client). Handled by `Src/microros_atomic64.c`. `RCUTILS_NO_64_ATOMIC=ON` (in the utils'
   `colcon.meta`) covers `rcutils` only -- `rcl` still needs the shim. (Maintainer-confirmed:
   micro_ros_stm32cubemx_utils#112.)
Other unresolved symbols at link (`_sbrk`, `clock_gettime`, `usleep`) are **app-provided**:
`clock_gettime` comes from the utils' `microros_time.c`; `usleep` + `clock_gettime` glue is
in `Src/microros_glue.c`; `_sbrk` etc. from `-specs=nosys.specs`.

## 7.5. The two runtime gotchas (why F3 was hard)

These bite at *run* time (the firmware links and "boots") and were the Plan-4 blockers:
1. **FreeRTOS task priority `< configMAX_PRIORITIES`.** `configMAX_PRIORITIES` is 7 here, so
   valid priorities are 0-6. `osPriorityNormal` (24) from CMSIS-RTOS is **not** valid in raw
   FreeRTOS -- `xTaskCreate(...,24,...)` trips `configASSERT(uxPriority < configMAX_PRIORITIES)`,
   which disables interrupts and spins **inside `xTaskCreate`, before the scheduler starts**.
   The fix is `configMAX_PRIORITIES - 2`. This is why the F2 smoke checks *liveness*, not just
   "no fault" (section 6).
2. **Renode socket terminals default to TELNET.** `emulation CreateServerSocketTerminal <port>
   "name"` injects telnet IAC negotiation and escapes `0xFF`, corrupting the **binary** XRCE
   stream both ways -> the agent never sees a session. Pass the 3rd arg `false`
   (`emitConfigBytes=false`) for a raw socket.

## 8. CI

`.github/workflows/stm32-f446re.yml` runs on every push **and pull request** touching
`test_host/`, `firmware/lib/`, `firmware_stm32/`, the root `Makefile`, or the workflow itself
(the F6 job additionally is gated to push events only). Three jobs:
- **host-tests** -> `make test-host` (Tier A).
- **stm32-f0-f2** -> `make libmicroros` + `make build-fw` (Tier B) -> `renode-test-action` runs
  `boot_smoke.robot` (F2, Renode 1.16.1) -> installs Renode-portable + socat -> `make control`
  (F4) -> `make imu` (F5).
- **stm32-topics** (`needs: stm32-f0-f2`, push-gated, `continue-on-error` until proven stable)
  -> `make libmicroros` + `make build-fw` -> `make topics` (F6, full base-node round-trip via the
  Docker micro_ros_agent). This is the only Docker round-trip that runs in CI (F3 is local-only).
This is separate from the existing PlatformIO CI (`.github/parse_platformio.py`), which only
sees `firmware/platformio.ini` envs.

## 9. Repo map (what builds what)

```
Makefile                      # the entry point (this guide's targets)
test_host/                    # Tier A: host unit tests (doctest)
firmware/lib/{kinematics,pid,odometry,odom_integrator}/   # portable math (shared)
firmware_stm32/               # Tier B/C: the GUI-free native firmware
  Makefile                    #   hand-written build (+ print_cflags for the Docker builder)
  STM32F446RETX_FLASH.ld      #   linker script
  Inc/{FreeRTOSConfig,stm32f4xx_hal_conf}.h
  Src/{main,microros_glue,microros_atomic64,sys_freertos,lino_hal,stm32_encoder,
       stm32_motor,stm32_i2c,control_loop,cxx_runtime}.{c,cpp}   # C main + C++ drivers/control
  Inc/{encoder_math,pwm_timing,imu_math,control_core,lino_hal,stm32_*,control_loop,...}.h
  config/{config.h,f446re_config.h}       #   board selector + F446RE descriptor tables
  vendor/                     #   pinned submodules: cmsis_device_f4, cmsis_core, HAL, FreeRTOS-Kernel
  micro_ros_stm32cubemx_utils #   pinned submodule (a5b2127); supplies extra_sources + the Docker builder
  renode/boot_smoke.{sh,robot}          # Tier C  (F2: firmware transmits the ping / scheduler ticked)
  renode/control_smoke.sh               # Tier C  (F4: control loop + injected-encoder->odom)
  renode/imu_smoke.sh + mpu6050_mock.py # Tier C  (F5: MPU6050 over HAL I2C, Python mock slave)
  renode/agent_bridge.sh                # Tier C+ (F3: live agent round-trip)
  renode/topic_roundtrip.sh             # Tier C+ (F6: full base-node cmd_vel/odom/imu round-trip)
docs/STM32CUBE_PORTING_PLAN.md            # the design spec (phases F0-F7)
docs/superpowers/plans/2026-06-*-*.md     # the per-phase implementation plans
```

## 10. Where this sits in the global plan

The port is a 7-plan sequence across phases F0-F7 (see `docs/STM32CUBE_PORTING_PLAN.md` section 8):
F1 host tests (Plan 1, **done**) | F0 link (Plan 2, **done**) | F2 boot (Plan 3, **done**) |
F3 transport+agent (Plan 4, **done** -- live round-trip in emulation) | F4 encoder/PWM (Plan 5,
**done** -- TIM encoder + PWM motor + 50 Hz control loop) | F5 IMU (Plan 6, **done** -- MPU6050 over HAL
I2C) | F6 full loop+topics+CI (Plan 7, **done** -- `/cmd_vel` + `/odom` + `/imu` over a live agent) | F7
hardware (on real NUCLEO-F446RE -- the only remaining phase). Tiers A/B/C/C+ correspond to
F1/F0/F2+F4+F5/F3+F6 -- the **emulation-provable port is complete**; everything to F6 is green in CI.
