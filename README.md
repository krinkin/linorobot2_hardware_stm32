# linorobot2_hardware — native STM32 (STM32Cube/HAL) port

A **GUI-free native STM32Cube/HAL port** of the linorobot2 low-level micro-ROS base controller,
living in **[`firmware_stm32/`](firmware_stm32/)**. It is the microcontroller-side firmware: it
subscribes to `/cmd_vel`, runs per-wheel PID over encoder feedback to drive the motors, reads an
IMU, and publishes `/odom/unfiltered` + `/imu/data_raw` back to ROS 2 over micro-ROS — a drop-in
replacement for the Arduino firmware's base node, but built directly on ST's HAL + FreeRTOS + CMSIS
with **no STM32CubeMX, no `.ioc`, no GUI** (everything is hand-written from pinned OSS submodules and
a hand-written `Makefile`).

First board: **NUCLEO-F446RE** (Cortex-M4F). Designed to be **universal across any FPU STM32**
(config-driven TIM/I2C/pin descriptor tables — a second board is a new config header, zero driver edits).

> The original Arduino/PlatformIO firmware (ESP32 / Pico / Teensy) is unchanged and documented in
> **[`README.upstream.md`](README.upstream.md)**; its sources live under [`firmware/`](firmware/).
> This is the `jazzy` branch; the native port is on branch `stm32-native-firmware`.

## Status

The **emulation-provable port (phases Ф0–Ф6) is complete and green** (Renode + Docker micro-ROS agent,
no hardware). The only remaining phase is **Ф7 — bring-up on a real NUCLEO-F446RE**.

| Phase | What works | `make` target | Tag |
|---|---|---|---|
| Ф1 | host unit tests (kinematics/pid/odom + encoder/pwm/imu/control math) | `test-host` | `stm32cube-p1-host-test-tier` |
| Ф0 | links real `libmicroros` (hard-VFP, 64-bit-atomics shim, no libstdc++) | `build-fw` | `stm32cube-f0-link-proven` |
| Ф2 | boots in Renode; FreeRTOS + transmits the micro-ROS ping | `renode` | `stm32cube-fullfw-gui-free` |
| Ф3 | live `micro_ros_agent` XRCE session (no board) | `agent-roundtrip` | `stm32cube-p4-agent-roundtrip` |
| Ф4 | encoder (TIM) + PWM motor + 50 Hz control loop + odometry | `control` | `stm32cube-p5-control` |
| Ф5 | MPU6050 IMU over real HAL I2C (Python mock slave in Renode) | `imu` | `stm32cube-p6-imu` |
| Ф6 | full base node: `/cmd_vel` + `/odom/unfiltered` + `/imu/data_raw` + reconnect | `topics` | `stm32cube-p7-topics` |
| Ф7 | **on real NUCLEO-F446RE hardware** | — | *pending* |

What Ф7 still needs (things emulation cannot prove): real x4 quadrature counting & PWM duty, the
MPU6050 burst data values, I2C SCL bus-recovery, a real magnetometer / MPU9250, and tuning.

## Build & test

**No host ROS install is required** — `libmicroros` is built by a self-contained Docker image, and
`ros2`/`micro_ros_agent` run only inside the agent container. There are two ways to run, pick one:

### Option 1 — host needs *only* Docker (recommended for a clean machine)

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
(`15:13.2.rel1-2`) — the same toolchain as CI and the dev box — so it reproduces the validated
firmware (`text=108896`). (Heads-up: the upstream ARM-official 13.2.Rel1 tarball reports the same
version but ships a different newlib-nano whose firmware hangs at boot in Renode — so we
deliberately use the distro build.)

### Option 2 — native tools on the host

```bash
git submodule update --init --recursive
# Tools: arm-none-eabi-gcc/g++ (13.2.x), GNU make, docker, socat, and Renode 1.16.1:
curl -L https://github.com/renode/renode/releases/download/v1.16.1/renode-1.16.1.linux-portable.tar.gz \
  | tar -xz -C "$HOME"                       # -> $HOME/renode_1.16.1_portable/renode
export RENODE="$HOME/renode_1.16.1_portable/renode"   # the smokes read $RENODE (or put it on PATH)

make test-all     # -> "================ ALL TIERS GREEN ================"
make help         # list every target
```

**Three-tier model (run individually — prefix any with `docker-` to run it in the image):**
```bash
# Tier A — pure host math, no MCU toolchain:
make test-host

# Tier B — link the F446RE firmware (needs docker + arm-none-eabi-gcc):
make libmicroros  # build libmicroros.a via the pinned Docker image (~once; slow)
make build-fw     # -> firmware_stm32/build/firmware_stm32.elf

# Tier C — Renode emulation (no board; needs Renode + socat):
make renode       # Ф2: boots + transmits the ping
make control      # Ф4: control loop + injected-encoder -> odometry
make imu          # Ф5: MPU6050 over HAL I2C via a Python mock

# Tier C+ — with a live micro-ROS agent (renode + docker + socat):
make agent-roundtrip   # Ф3: XRCE session
make topics            # Ф6: full cmd_vel/odom/imu round-trip
```

CI: `.github/workflows/stm32-f446re.yml` runs Tiers A/B + Ф2/Ф4/Ф5 on every push and PR, and Ф6
(Docker agent) on push.

## Repo layout (native port)

```
Makefile                       # the entry point (targets above)
docker/Dockerfile              # self-contained dev image (toolchain + Renode + socat) for `make docker-*`
test_host/                     # Tier A host doctest tier (see test_host/README.md)
firmware/lib/{kinematics,pid,odom_integrator,imu,motor}/   # portable code reused by both ports
firmware_stm32/                # the native HAL port
  Src/  Inc/  config/          # C main + C++ drivers/control; board descriptor tables
  vendor/                      # pinned submodules: cmsis_device_f4, cmsis_core, HAL, FreeRTOS-Kernel
  micro_ros_stm32cubemx_utils  # pinned submodule (a5b2127) — Docker libmicroros builder (no GUI needed)
  renode/                      # boot/control/imu smokes + agent/topic round-trips + MPU6050 mock
docs/TESTING.md                # hands-on build/test guide (tiers, gotchas, CI)
docs/STM32CUBE_PORTING_PLAN.md # design spec (phases Ф0–Ф7)
docs/superpowers/plans/        # per-phase as-built plan docs
```

## More

- **[`docs/TESTING.md`](docs/TESTING.md)** — the full hands-on guide (what each tier proves, the two
  link gotchas, CI).
- **[`docs/STM32CUBE_PORTING_PLAN.md`](docs/STM32CUBE_PORTING_PLAN.md)** — the design spec.
- **[`docs/superpowers/plans/`](docs/superpowers/plans/)** — per-phase as-built records.
- The matching ROS 2 nodes live in the separate [linorobot2](https://github.com/linorobot/linorobot2) repo.
