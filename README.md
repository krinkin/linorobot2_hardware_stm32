# linorobot2_hardware_stm32 -- native STM32 micro-ROS base controller

[![CI](https://github.com/krinkin/linorobot2_hardware_stm32/actions/workflows/stm32-f446re.yml/badge.svg?branch=stm32)](https://github.com/krinkin/linorobot2_hardware_stm32/actions/workflows/stm32-f446re.yml)

A native **STM32Cube/HAL** micro-ROS firmware for the linorobot2 low-level base controller, in
[`firmware_stm32/`](firmware_stm32/). It subscribes to `/cmd_vel`, runs per-wheel PID over encoder
feedback to drive the motors, reads an IMU, and publishes `/odom/unfiltered` + `/imu/data_raw` over
micro-ROS. It is built directly on ST HAL + FreeRTOS + CMSIS with **no STM32CubeMX, no `.ioc`, no
GUI** -- hand-written from pinned OSS submodules and a hand-written `Makefile`.

First board: **NUCLEO-F446RE** (Cortex-M4F). Config-driven TIM/I2C/pin descriptor tables keep it
portable across FPU STM32 parts -- a second board is a new config header, zero driver edits. It
targets ROS 2 **Jazzy**; the matching ROS 2 nodes live in the separate
[linorobot2](https://github.com/linorobot/linorobot2) repo. Default branch: `stm32`.

## Status

The firmware is complete and runs **end-to-end in emulation** (Renode + a live `micro_ros_agent`, no
board). The whole suite is green in CI -- `make test-all` (or `make docker-test-all`) covers:

- host unit tests for the kinematics / PID / odometry / encoder / PWM / IMU math;
- a real `libmicroros` link for F446RE (hard-VFP, 64-bit-atomics shim, no libstdc++);
- boot in Renode: FreeRTOS comes up and transmits the micro-ROS ping;
- the 50 Hz control loop -- encoder (TIM) -> PID -> PWM motor -> odometry;
- an MPU6050 read over the real HAL I2C path;
- the full base node over a live agent: `/cmd_vel` in, `/odom/unfiltered` + `/imu/data_raw` out, with
  agent reconnect.

**Remaining: bring-up and tuning on real NUCLEO-F446RE hardware** -- the parts emulation cannot prove
(real x4 quadrature counts and PWM duty, the MPU6050 burst values, I2C bus-recovery, a real
magnetometer / MPU9250, and gain tuning).

## Build & test

**No host ROS install is required** -- `libmicroros` is built by a self-contained Docker image, and
`ros2` / `micro_ros_agent` run only inside the agent container. There are two ways to run, pick one.

### Option 1 -- host needs *only* Docker (recommended for a clean machine)

Everything else (ARM toolchain, Renode, socat) lives in a self-contained dev image
([`docker/Dockerfile`](docker/Dockerfile)). You install nothing but Docker.

```bash
git submodule update --init --recursive    # CMSIS / HAL / FreeRTOS + micro_ros utils (pinned)
make docker-test-all                        # build the dev image, then run the WHOLE suite in it
# -> "================ ALL TIERS GREEN ================"

make docker-shell                           # interactive shell in the dev image
make docker-build-fw                        # run any single target in the image (make docker-<target>)
```

The dev image carries the toolchain + Renode; `make libmicroros` and the agent tiers run as
*sibling* containers via the bind-mounted host Docker socket (Docker-out-of-Docker), so the only
thing on your host is Docker itself. It uses Ubuntu 24.04's distro `gcc-arm-none-eabi`
(`15:13.2.rel1-2`) -- the same toolchain as CI -- so it reproduces the validated firmware
(`text=108896`). (The upstream ARM-official 13.2.Rel1 tarball reports the same version but ships a
different newlib-nano whose firmware hangs at boot in Renode, so we deliberately use the distro build.)

### Option 2 -- native tools on the host

```bash
git submodule update --init --recursive
# Tools: distro arm-none-eabi-gcc/g++ + newlib (13.2.x), GNU make, docker, socat, and Renode 1.16.1:
curl -L https://github.com/renode/renode/releases/download/v1.16.1/renode-1.16.1.linux-portable.tar.gz \
  | tar -xz -C "$HOME"                       # -> $HOME/renode_1.16.1_portable/renode
export RENODE="$HOME/renode_1.16.1_portable/renode"   # the smokes read $RENODE (or put it on PATH)

make test-all     # -> "================ ALL TIERS GREEN ================"
make help         # list every target
```

**Run a single tier (prefix any with `docker-` to run it in the image):**
```bash
make test-host         # host unit tests (no MCU toolchain)
make libmicroros       # build libmicroros.a via the pinned Docker image (~once; slow)
make build-fw          # link the firmware -> firmware_stm32/build/firmware_stm32.elf
make renode            # boot smoke in Renode (firmware transmits the ping)
make control           # 50 Hz control loop + injected-encoder -> odometry (Renode)
make imu               # MPU6050 over HAL I2C via a Python mock (Renode)
make agent-roundtrip   # live micro-ROS agent XRCE session (Renode + Docker agent)
make topics            # full cmd_vel/odom/imu round-trip over a live agent
```

CI (`.github/workflows/stm32-f446re.yml`, the only workflow) runs the host tests, the firmware link,
and the Renode boot/control/IMU smokes on every push and PR; the full topic round-trip on push.

## Architecture

Two FreeRTOS tasks:
- a **control task** -- the 50 Hz loop: `kinematics.getRPM(cmd_vel)` -> per-wheel
  `PID.compute(target, encoder.getRPM())` -> `motor.spin(pwm)` -> odometry, plus the IMU read. It runs
  regardless of the agent and brakes the base on a 200 ms `/cmd_vel` deadman.
- a **micro-ROS task** -- the rclc executor + a 4-state agent-reconnect machine that recreates
  entities on agent loss and publishes `/odom/unfiltered` + `/imu/data_raw`.

The two communicate through a small plain-C snapshot surface under a critical section -- no C++ type
crosses the task boundary. HAL/CMSIS symbols are confined to the `Src/stm32_*.cpp` + `lino_hal`
layer; everything else is platform-agnostic and host-tested. Board specifics (geometry, PID gains,
TIM/I2C/pin tables) live in `firmware_stm32/config/`. See [`CLAUDE.md`](CLAUDE.md) and
[`docs/TESTING.md`](docs/TESTING.md) for the full picture.

## Repo layout

```
Makefile                       # the entry point (make help)
docker/Dockerfile              # self-contained dev image (toolchain + Renode + socat) for make docker-*
test_host/                     # host unit-test tier (doctest)
firmware/lib/{kinematics,pid,odom_integrator}/   # platform-agnostic libs reused by the firmware
firmware/lib/imu/{imu,mag}_interface.h           # IMU/MAG interfaces (the firmware has native drivers)
firmware_stm32/                # the native HAL firmware
  Src/  Inc/  config/          # C main + C++ drivers/control; board descriptor tables
  vendor/                      # pinned submodules: cmsis_device_f4, cmsis_core, HAL, FreeRTOS-Kernel
  micro_ros_stm32cubemx_utils  # pinned submodule -- Docker libmicroros builder (no GUI needed)
  renode/                      # Renode smokes + agent/topic round-trips + MPU6050 mock
docs/TESTING.md                # hands-on build/test guide
```

## More

- **[`docs/TESTING.md`](docs/TESTING.md)** -- the hands-on guide: what each tier proves, the link
  gotchas, and the Renode emulation caveats.
- **[`CLAUDE.md`](CLAUDE.md)** -- architecture and repo conventions.
- The matching ROS 2 nodes live in the separate
  [linorobot2](https://github.com/linorobot/linorobot2) repo.
- Design notes and the as-built record of how the firmware was built are kept for history in
  [`docs/STM32CUBE_PORTING_PLAN.md`](docs/STM32CUBE_PORTING_PLAN.md) and
  [`docs/superpowers/plans/`](docs/superpowers/plans/).
