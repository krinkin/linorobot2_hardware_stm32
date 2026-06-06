# Plan to port linorobot2_hardware to native STM32Cube (HAL) -- a reliable plan

> **STATUS (2026-06-06): phases F0-F6 IMPLEMENTED and green in emulation** (tags `stm32cube-p1-host-test-tier`
> ... `stm32cube-p7-topics`). The port is merged into the **default branch `stm32`** of the `linorobot2_hardware_stm32` repository.
> `firmware_stm32/` builds; `make test-all` passes (host + libmicroros + build-fw + F2 + F4 + F5),
> F3/F6 verified via a live agent. The whole set is also reproducible **under Docker only**:
> `make docker-test-all` (see `docker/Dockerfile`). **The only remaining phase is F7: bring-up on
> a real NUCLEO-F446RE.** Below is the original design-spec (historical basis); per-phase specifics are
> in `docs/superpowers/plans/2026-06-0*-stm32cube-*.md` and `docs/TESTING.md`.

> This is the design-spec for the native STM32Cube path (ST HAL/LL + CMSIS, without the Arduino layer)
> -- the approach that was actually built. An earlier minimal-diff Arduino/STM32duino variant was
> considered but not pursued. This document is a design-spec; the detailed per-phase implementation
> plans live under `docs/superpowers/plans/`.

## 0. Summary and locked-in decisions

**Goal:** port the firmware of the linorobot2 low-level controller to native STM32Cube so that the
code is **universal across any STM32** (with a caveat: the practical floor is a chip with an FPU and >= ~64 KB RAM;
F0/L0/G0/F1-BluePill are excluded -- micro-ROS does not fit there).

**The micro-ROS integration fork is resolved in favor of the official native path (Route B):**
project: **native HAL/FreeRTOS + hand-written Makefile (GUI-free, no CubeMX)** + the official **`micro_ros_stm32cubemx_utils`** (branch `jazzy`),
`libmicroros.a` is built by the Docker image `microros/micro_ros_static_library_builder:jazzy`.

| Decision | Choice |
|---|---|
| micro-ROS integration | Route B (native HAL + hand-written Makefile, **GUI-free**, + `micro_ros_stm32cubemx_utils`, branch jazzy) |
| libmicroros build | Docker `microros/micro_ros_static_library_builder:jazzy` (ABI-match by construction) |
| Execution model | FreeRTOS / CMSIS-OS v2; rclc-executor in a task with a stack >= 24 KB |
| Transport | UART, compile-time switch **DMA** (hardware/Renode) <-> **IT/polling** (QEMU) |
| IMU/MAG | MPU6050 / MPU9250 via I2Cdevlib (QMI8658/QMC5883L out of scope -- not selected) |
| Development | **Emulation-first** (no hardware): Renode (primary) + host unit tests + QEMU (optional) |
| First target | **NUCLEO-F446RE** (M4F, 128 KB RAM / 512 KB Flash; community-validated, best coverage in emulators) |
| Cost | All mandatory tools are free (see section 11) |

**Bottom line on "no hardware":** ~70 % of the work is done and verified without a board (the math via host tests,
MCU integration and transport in Renode); the "analog/timing tail" (real PWM frequency, encoder physics,
MPU bring-up, framing@921600, jitter) is validated only on the chip later (section 10).

---

## 1. Architecture: 3 layers as the testability boundary

The main principle of the port is to **strictly separate the portable from the platform-specific**, so that the maximum amount of code is verified
without hardware.

```
+----------------------------------------------------------------+
| Layer 3: micro-ROS application + FreeRTOS                       |
|  - port of the 4-state reconnect machine into a task (modeled   |
|    on sample_main.c), rclc executor, 50 Hz timer, deadman 200 ms|
|  - verified by: Renode / QEMU                                   |
+----------------------------------------------------------------+
| Layer 2: thin peripheral HAL layer (NEW native code)           |
|  - encoder (TIM encoder mode), motor (TIM PWM + GPIO dir),      |
|    I2Cdev backend (HAL_I2C_Mem_*), micro-ROS UART transport,    |
|    timebase millis/micros, minimal Serial/Wire shims            |
|  - verified by: Renode (partially) -> hardware (fully)          |
+----------------------------------------------------------------+
| Layer 1: portable core (reused as-is)                          |
|  - kinematics, pid, odometry(*) -- Arduino types + timebase only|
|  - verified by: HOST unit tests on a PC (no emulator)          |
+----------------------------------------------------------------+
(*) odometry needs a targeted refactor -- see section 5.6
```

The repository's compile-time HW abstraction (`USE_*` macros, `#define Motor/Encoder/IMU`, hooks
`BOARD_INIT/BOARD_INIT_LATE/BOARD_LOOP`) is **preserved** and extended with emulation/Fake targets
(scripted-encoder, FakeIMU) -- in the spirit of the existing `USE_FAKE_IMU`.

---

## 2. The micro-ROS integration route (B) and why not A

A general hard fact: the library the repository uses now (`micro_ros_platformio`)
is **structurally only for `framework=arduino`** (it contains only the `platform_code/arduino/` directory).
This means auto-cross-compilation of `libmicroros` "like on ESP32/Pico" on native Cube **will not happen**;
`libmicroros.a` is built separately.

**Route A -- PlatformIO `framework=stm32cube` + a manually linked `libmicroros.a`. REJECTED.**
The single public precedent (Nucleo-F446RE) got past the 64-bit atomics, but **was dead-stopped at
`libmicroros.a uses VFP register arguments` -- a hard-float ABI mismatch** between the Docker build and
the PlatformIO toolchain. There is no reproducible build.

**Route B -- native HAL/FreeRTOS + hand-written Makefile (GUI-free) + `micro_ros_stm32cubemx_utils`. CHOSEN.**
- A live `jazzy` branch (matches this repo's "distro = branch" model).
- **ABI matches by construction:** the Docker builder takes the application's real CFLAGS via
  `make print_cflags` and passes them into colcon -> no VFP wall of Route A.
- A stable transport contract: `rmw_uros_set_custom_transport(true, &huartX, open, close, write, read)`.
- `colcon.meta` already sets the needed STM32 keys: `RMW_UXRCE_MAX_NODES=1`, `MAX_PUBLISHERS=10`,
  `MAX_SUBSCRIPTIONS=5`, `RMW_UXRCE_TRANSPORT=custom`, `RCUTILS_NO_64_ATOMIC=ON` -- but it covers
  **only `rcutils`**; `rcl` (time/timer/client) uses 64-bit atomics unconditionally, so on
  Cortex-M you additionally need **your own `__atomic_*_8` shim** (empirically confirmed -- see Plan 2 Task 5a).
- Cost: **FreeRTOS** is assumed (all examples and the transport `.c` files pull in `cmsis_os.h`) and a **separate CI**
  with the Docker build of libmicroros (PlatformIO's `parse_platformio.py` does not see it).

---

## 3. Repository structure

- A new directory **`firmware_stm32/`** (hand-written HAL/FreeRTOS project + Makefile, **no CubeMX/GUI**) **next to** `firmware/`, not instead of it.
- The portable libs (`firmware/lib/{kinematics,pid,odometry}`) are pulled in **by include reference**, not
  forked -- single source of truth between the Arduino and Cube builds.
- `micro_ros_stm32cubemx_utils` is vendored (submodule or a copy with a pinned commit).
- `libmicroros.a` + `microros_include/` are committed as an **artifact lockfile**; the Docker image is pinned by
  digest; `extra_packages`/`colcon.meta` are fixed for reproducibility.
- **`I2Cdevlib-Core` is forked/vendored** (its files are pulled via lib_deps and cannot be edited in-repo) -- a HAL backend is added to the fork (section 5.3). The sensor classes (MPU6050/MPU9250) meanwhile
  **do not change**.

---

## 4. Scope of work by component

| Component | File(s) | Status | What we do |
|---|---|---|---|
| Kinematics | `firmware/lib/kinematics/*` | [OK] as-is | timebase shim only |
| PID | `firmware/lib/pid/*` | [OK] as-is | timebase shim only |
| Odometry | `firmware/lib/odometry/*` | (edit) refactor | extract a pure integrator without the `nav_msgs` dependency (section 5.6) |
| IMU classes (MPU6050/9250) | `firmware/lib/imu/default_imu.h` | [OK] as-is | run on top of the new I2Cdev backend |
| I2Cdev primitives | fork of `I2Cdevlib-Core` | (rewrite) | ~10 methods -> `HAL_I2C_Mem_Read/Write` (section 5.3) |
| Encoder | `firmware/lib/encoder/encoder.h` | (rewrite) | TIM encoder mode (section 5.1) |
| Motor/PWM | `firmware/lib/motor/default_motor.h` | (rewrite) | TIM PWM + GPIO direction (section 5.2) |
| Transport | `firmware/src/firmware.ino:183` | (rewrite) | custom transport DMA/IT (section 5.4) |
| Platform glue | `firmware/src/firmware.ino` (setup/loop, Serial, millis...) | (shim) | -> HAL/FreeRTOS task (section 5.5) |
| Board config | `firmware_stm32/.../f446re_config.h` | (new) | pin-map -> concrete `TIMx_CHy` + AF |
| Build | `firmware_stm32/Makefile` (hand-written, GUI-free) | (new) | Makefile flow of micro_ros_stm32cubemx_utils |
| CI | `.github/workflows/stm32-f446re.yml` | (new) | separate workflow (section 7) |

**Out of scope (not selected by the user):** `QMI8658` and `default_mag.h`/QMC5883L (direct Wire) -- separate
Wire->HAL rewrites; not done while IMU = MPU6050/9250.

---

## 5. Specifics of the native rewrites

### 5.1 Encoder -- TIM encoder mode
- `TIM_ENCODERMODE_TI12` (x4 decoding), phases A/B = **CH1/CH2 of the same TIMx** (cannot be split across
  two timers). Read -- `TIMx->CNT` directly; direction -- the `DIR` flag.
- 32-bit timers (**TIM2/TIM5**) are preferred where absolute position matters; 16-bit ones are correct
  only via a **delta computation** between polls (at 50 Hz a wrap does not occur in time), which is what the current
  `getRPM()` formula does. WARNING: the current code stores raw monotonic ticks -- a naive port to a 16-bit CNT
  must handle wrap explicitly (subtraction modulo).
- The API is preserved: `Encoder(int p1,int p2,int cpr,bool invert)`, `float getRPM()`, `int32_t read()`,
  `void write(int32_t)`.

### 5.2 Motor/PWM -- TIM PWM mode
- `HAL_TIM_PWM_Start(&htimX, TIM_CHANNEL_y)` + `__HAL_TIM_SET_COMPARE(...)`. 20 kHz @ 10-bit ->
  `ARR = 1023` + a tuned `PSC` from the timer clock.
- WARNING: PWM frequency is **per-timer (shared ARR), not per-pin**: for BTS7960 both PWM pins must be two
  channels of the same timer. Direction -- GPIO via `HAL_GPIO_WritePin`.
- ESC path (`Servo`) -> a TIM channel at 50 Hz with a 1000-2000 us pulse.

### 5.3 I2Cdev backend
- Port **only ~10 static primitives** `I2Cdev::readByte/writeByte/readBytes/readBit(s)/
  writeBit(s)` on top of `HAL_I2C_Mem_Read/Write`. All 8 device classes (MPU6050/9250, ADXL345, ITG3200,
  HMC5883L, AK89xx) call only this static interface and stay **byte-for-byte** the same.
- `I2Cdev.cpp` uses `millis()` for read timeouts -> a timebase shim is needed (section 5.5).
- WARNING: done in a **fork** of I2Cdevlib-Core (see section 3).

### 5.4 micro-ROS transport -- a compile-time switch
- Replace `set_microros_serial_transports(Serial)` with
  `rmw_uros_set_custom_transport(true, &huartX, cubemx_transport_open/close/write/read)`.
- Ready-made files from `micro_ros_stm32cubemx_utils`: `dma_transport.c` (UART/DMA, recommended),
  `it_transport.c` (UART/IT), `usb_cdc_transport.c`, `udp_transport.c`.
- **Key choice for emulation:** DMA -- on hardware and in Renode; IT/polling -- in the QEMU build (DMA is not
  modeled in QEMU). The same logic in all three places via a build flag.
- WARNING: **do not select USB-CDC** -- it pulls in the proprietary `STM32_USB_Device_Library` (SLA0044); UART stays
  fully BSD-3/Apache-2.0 (see section 11).

### 5.5 Platform glue (shim)
- `millis()` -> FreeRTOS tick (`HAL_GetTick`/`xTaskGetTickCount`); `micros()` -> free-running 1 MHz **TIM5**
  counter (NOT DWT -- Renode does not model the DWT cycle counter); `delay()` -> `vTaskDelay`;
  `pinMode/digitalWrite/digitalRead` -> `HAL_GPIO_*`; `Serial` (logs) -> `HAL_UART` or ITM/semihosting.
- `setup()/loop()` -> initialization in `main()` + rclc-executor in a FreeRTOS task (stack >= 24 KB).

### 5.6 odometry refactor (for host testability)
- Extract a pure position integrator
  (`x += (vx*cos(theta) - vy*sin(theta))*dt; y += (vx*sin(theta) + vy*cos(theta))*dt; theta += wz*dt`)
  into a function **without** a dependency on `nav_msgs__msg__Odometry` / `micro_ros_string_utilities`, so that
  the host test does not link the rosidl runtime. The publication wrapper stays in Layer 3.

---

## 6. Emulation without hardware (3 tiers) + a "what-proves-what" matrix

- **Tier A -- host unit tests (fast, every commit):** `kinematics`, `pid`, the extracted integrator
  `odometry` under PlatformIO `native`/GoogleTest with an `Arduino.h` shim (`constrain/fabs/PI`). The fastest
  and most reliable gate for the math; no emulator needed.
- **Tier B -- Renode (the main integration gate):**
  - The `.repl` for F446RE models all boot peripherals (RCC/FLASH/NVIC/SysTick/USART/DMA/TIM/I2C) -- otherwise
    `HAL_Init` hangs/HardFaults (the top emulation risk).
  - Agent: either an in-sim hub (as in `antmicro/renode-microros-demo`), or
    `CreateUartPtyTerminal` -> host `micro_ros_agent serial --dev /tmp/uart` (this path is a spike: there is no public
    end-to-end example of host-agent-over-Renode-pty).
  - Headless asserts: Robot Framework `Wait For Line On Uart  <regex>  treatAsRegex=true`; via
    `antmicro/renode-test-action`, with a pinned `renode-revision`.
  - Encoder -- scripting CH1/CH2 edges; IMU -- a custom I2C slave (there is no ready-made MPU model in Renode)
    **or** FakeIMU.
- **Tier C -- QEMU (optional):** `qemu-system-arm -M netduinoplus2 -semihosting` boot+session-smoke with
  FakeIMU and the IT transport. WARNING: the `netduinoplus2` machine is **F405-class, not F446RE exactly**: this is a smoke test
  at the Cortex-M4 core level, not board-accurate (a separate QEMU build for an available machine). WARNING:
  `docker/setup-qemu-action` does **not** install `qemu-system-arm` (only user-mode binfmt) -- an explicit
  `apt-get install qemu-system-arm` is needed.

**"What-proves-what" matrix:**

| Loop piece | Host test | QEMU | Renode | Hardware only |
|---|---|---|---|---|
| `kinematics.getRPM` mixing/saturation | [OK] primary | -- | -- | -- |
| `PID.compute` saturation/anti-windup | [OK] primary | -- | -- | -- |
| `odometry` integrator | [OK] (after refactor) | -- | -- | -- |
| 50 Hz rcl timer (logic) | -- | [OK] | [OK] | real jitter |
| reconnect machine + session + round-trip | -- | [OK] (IT) | [OK] (incl. DMA) | framing@921600 |
| FreeRTOS stack + `rclc_support_init` | -- | [OK] (medium) | [OK] (medium) | worst-case high-water-mark |
| Encoder: count/direction/scale | -- | [FAIL] | [OK] (edge scripting) | CPR, sign at max RPM, 16-bit wrap, dynamics |
| IMU I2C plumbing + publication | -- | [FAIL] | [OK] (needs I2C slave) | MPU bring-up, `calibrateGyro` |
| PWM duty/channel + GPIO direction | -- | [FAIL] | WARNING: registers, not the waveform | 20 kHz/10-bit on a scope, sign |
| UART/DMA reliability (RX-to-idle) | -- | [FAIL] | [OK] functional | framing under load |

WARNING: Both emulators are **not timing/baudrate accurate** (Renode UART -- a byte queue; QEMU USART does not honor
bit-level timing): baud-rate mismatch, framing-under-load, and interrupt latency are **not reproducible
in either of them**.

---

## 7. CI

A separate `.github/workflows/stm32-f446re.yml` (does not touch `parse_platformio.py`, the existing matrix does not
break):
1. `docker run microros/micro_ros_static_library_builder:jazzy` -> `libmicroros.a` (non-interactively: either
   pre-seed stdin, or the IDE/Make variant; there is no upstream CI for it -- we write our own).
2. `make` build of `firmware_stm32` -> `.elf`.
3. Renode robot tests (`antmicro/renode-test-action`, pin revision) -> the main gate.
4. (optional) QEMU boot/session-smoke.
- **Pinning:** `antmicro/renode:1.16.1`, `microros/micro-ros-agent:jazzy` (digest re-resolve),
  QEMU from apt Ubuntu 24.04, the builder Docker image by sha256.
- **Docker Hub limits:** `docker/login-action` or a GHCR mirror (throttling, not cost -- section 11).

---

## 8. Phases (milestones)

- **F0 -- skeleton:** the F446RE project (hand-written HAL/FreeRTOS, GUI-free) compiles, `libmicroros.a` links (**ABI-smoke** --
  **ABI-smoke = TWO gates**: VFP **and** 64-bit atomics/POSIX. Empirically: VFP cleared (all 2014 members hard-float); atomics require a `__atomic_*_8` shim + `usleep` (Plan 2 Task 5a)).
- **F1 -- host tests:** `kinematics/pid/odometry` integrator green on a PC.
- **F2 -- Renode boot:** FreeRTOS starts, `rclc_support_init` without a HardFault (stack 24-32 KB).
- **F3 -- Renode transport:** micro-ROS session to the agent, round-trip.
- **F4 -- Renode encoder/PWM:** encoder logic (edge scripting) + register asserts of PWM.
- **F5 -- IMU:** FakeIMU -> a custom Renode I2C slave -> `/imu/data_raw` is published.
- **F6 -- Renode full loop:** `cmd_vel`->kinematics->PID->motor, `/odom`+`/imu` at 50 Hz.
- **F7 -- hardware (when a board is available):** 6 residual items (section 10); then duplicating the env for other
  STM32 families (universality).

---

## 9. Risks (ranked) and mitigations

1. **Renode `.repl` authoring** for F446RE -- the top emulation risk. -> take a ready-made `.repl` precedent
   for F446RE (prdktntwcklr/renode-example), grow the peripherals incrementally.
2. **Float-ABI/FPU** -- removed by Route B (Docker `make print_cflags`); verified by ABI-smoke in F0. Empirically: VFP cleared (2014/2014 hard-float). **BUT** on Cortex-M a real blocker remains: `rcl` pulls in `__atomic_*_8`, which do not exist in arm-none-eabi (baremetal); `RCUTILS_NO_64_ATOMIC=ON` covers only `rcutils`. -> PRIMASK `__atomic_*_8` shim + `usleep` (Plan 2 Task 5a; micro_ros_stm32cubemx_utils#112).
3. **FreeRTOS stack / HardFault on `rclc_support_init`** (the sample 12 KB stack is too small). -> 24-32 KB, high-water-mark
   + static analysis; be careful with the custom allocator (it may conflict with the stack).
4. **No MPU model in Renode** -- a custom I2C slave (WHO_AM_I + registers) or FakeIMU until hardware.
5. **Emulators are not timing/baudrate accurate** -- framing@921600, 50 Hz jitter, real PWM frequency ->
   hardware only (F7).
6. **Universality beyond F4** (G4/H7/L4) -- may require authoring Renode `.repl`/models;
   H7 additionally needs MPU cache for UDP. -> "universal" = a goal after the green F4 path.
7. **libmicroros reproducibility** -- floating branch-HEAD/Docker tags. -> pin the image by digest, commit
   the `.a` as a lockfile, fix `extra_packages`/`colcon.meta`.

---

## 10. Residual -- validated on hardware only (F7)

1. **PWM** -- real frequency/resolution (20 kHz @ 10-bit) on an oscilloscope.
2. **Encoder** -- real `COUNTS_PER_REV`, direction sign, behavior at `MOTOR_MAX_RPM`, 16-bit wrap;
   the closed-loop dynamics of PID/odometry (there is no physical model in emulation).
3. **IMU** -- bring-up of MPU6050/9250 (WHO_AM_I, on MPU9250 -- AK8963 passthrough), `calibrateGyro` against
   real noise. WARNING: `imu.init()` is fatal in `setup()` -- this is a real gate.
4. **UART framing @ 921600** under DMA load (emulators are not baud/timing-accurate).
5. **Control-loop jitter** -- the 50 Hz timer and the 200 ms deadman under real bus/DMA load.
6. **RAM headroom** worst-case on a specific chip (a clean run in emulation reduces but does not remove the risk of
   an F4 HardFault).

---

## 11. Cost and licenses -- all free (verified)

**Conclusion:** all mandatory tools are free under three conditions that the plan already meets:
**a public repository**, **CI on Linux + Docker Engine** (not Desktop), **UART transport** (not USB-CDC).

| Tool | License | Free? |
|---|---|---|
| STM32Cube HAL/LL + BSP | BSD-3-Clause | [OK] |
| CMSIS Core + Device | Apache-2.0 | [OK] |
| arm-none-eabi-gcc / GNU Make | GPL (+GCC-exception) | [OK] |
| STM32CubeMX/IDE/Programmer | SLA0048 (proprietary freeware) | NO **NOT used** (the port is GUI-free) |
| micro_ros_stm32cubemx_utils, rcl/rclc/rmw, Micro XRCE-DDS, ROS 2 Jazzy | Apache-2.0 | [OK] |
| Renode / renode-test-action / Robot Framework | MIT / Apache-2.0 | [OK] |
| QEMU | GPL-2.0 | [OK] |
| GoogleTest / PlatformIO Core / socat | BSD / Apache-2.0 / GPL-2.0 | [OK] |
| FreeRTOS kernel / CMSIS-RTOS v2 | MIT / Apache-2.0 | [OK] |
| jrowberg I2Cdevlib | MIT | [OK] |
| Docker Engine (Linux) | Apache-2.0 | [OK] |
| Docker Desktop | DSSA (proprietary) | WARNING: paid for 250+ employees/>=$10M -- **not used** |
| GitHub Actions | SaaS | WARNING: public repo unlimited; private = 2000 min/month |
| STM32_USB_Device_Library (if USB-CDC) | SLA0044 (proprietary) | WARNING: **avoided -- we take UART** |

**Watch-outs (how to stay in the free lane):**
1. In CI -- **Docker Engine** on Linux runners, not Docker Desktop.
2. The repository is **public** -> Actions are free without limit (otherwise 2000 min/month or self-hosted).
3. Docker Hub pull limits -- `docker/login-action`/a GHCR mirror (throttling, not cost).
4. **ST GUI tools (CubeMX/IDE) are not used at all** -- the project is hand-written from **BSD-3/Apache-2.0** sources
   HAL/CMSIS + FreeRTOS (MIT) + gcc; no CubeMX/`.ioc`/GUI. This is what gives full CI reproducibility.
5. **UART transport** bypasses the proprietary SLA0044 (`STM32_USB_Device_Library`) -- an extra argument for UART.

No tool replacement **is needed** -- only these configuration choices are required (all already in the plan).

---

## 12. Open questions / what to confirm along the way

- The exact F446RE pin-map: 4 pairs of `TIMx_CH1/CH2` for the encoders + the motors' PWM channels + I2C(IMU) + UART
  (micro-ROS), without alternate-function collisions. Check against the datasheet.
- The drivetrain (2 vs 4 wheels) -- sets the timer budget; F446RE has TIM1/2/3/4/5/8 -- enough for 4 wheels,
  but the pin-map requires validation.
- ~~The host-agent-over-Renode-pty spike (no public end-to-end example) -- or go straight to an in-sim hub.~~
  **RESOLVED:** a live `micro_ros_agent` <-> firmware works via Renode RAW-socket <-> socat <-> agent in a single
  `--network host` container (`firmware_stm32/renode/agent_bridge.sh`, `topic_roundtrip.sh`; F3/F6 green).

---

## Sources (verified by reconnaissance)

- micro_ros_stm32cubemx_utils (branches humble/jazzy/...; the Docker builder; transports; colcon.meta) --
  https://github.com/micro-ROS/micro_ros_stm32cubemx_utils
- jazzy colcon.meta --
  https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/blob/jazzy/microros_static_library/library_generation/colcon.meta
- jazzy sample_main.c (the custom transport contract) --
  https://raw.githubusercontent.com/micro-ROS/micro_ros_stm32cubemx_utils/jazzy/sample_main.c
- dma_transport.c / it_transport.c --
  https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/humble/extra_sources/microros_transports
- micro_ros_platformio (framework=arduino only) -- https://github.com/micro-ROS/micro_ros_platformio
- PlatformIO Community: the VFP/float-ABI wall of Route A --
  https://community.platformio.org/t/stm32-framework-with-micro-ros-stm32cubemx-utils-undefined-references-to-sync-synchronize/42180
- micro_ros_stm32cubemx_utils #110 (24-32 KB stack, HardFault on init) --
  https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/issues/110
- Renode encoder modes (STM32_Timer.cs) --
  https://github.com/renode/renode-infrastructure/blob/master/src/Emulator/Peripherals/Peripherals/Timers/STM32_Timer.cs
- Renode I2C models (STM32F1_I2C.cs / STM32F7_I2C.cs) --
  https://github.com/renode/renode-infrastructure/tree/master/src/Emulator/Peripherals/Peripherals/I2C
- antmicro/renode-microros-demo + blog --
  https://github.com/antmicro/renode-microros-demo ,
  https://renode.io/news/fully-deterministic-linux-zephyr-micro-ros-testing-in-renode/
- renode-test-action / pinning --
  https://renode.io/news/renode-github-action-for-automated-testing-in-simulation/
- QEMU STM32 (what is modeled/not) -- https://www.qemu.org/docs/master/system/arm/stm32.html
- setup-qemu-action does not install qemu-system-arm -- https://github.com/docker/setup-qemu-action
- STM32 HAL = BSD-3-Clause; CMSIS = Apache-2.0; ST SLA0048 (freeware); SLA0044 (USB middleware);
  FreeRTOS = MIT; Renode = MIT; Docker Engine = Apache-2.0 vs Docker Desktop DSSA.
- ST AN4013 (timers/encoder mode) --
  https://www.st.com/resource/en/application_note/an4013-introduction-to-timers-for-stm32-mcus-stmicroelectronics.pdf

---

## Affected files (absolute paths)

**New:**
- `/home/claude/Projects/linorobot2_hardware/firmware_stm32/` -- hand-written HAL/FreeRTOS project + `Makefile` (GUI-free, no `.ioc`)
- `/home/claude/Projects/linorobot2_hardware/firmware_stm32/.../f446re_config.h` -- board config (pin-map)
- `/home/claude/Projects/linorobot2_hardware/.github/workflows/stm32-f446re.yml` -- separate CI
- fork/vendor of `I2Cdevlib-Core` with a HAL backend
- host tests (PlatformIO `native`/GoogleTest) for the kinematics/pid/odometry integrator
- Renode `.repl` + Robot tests

**Modified:**
- `firmware/lib/odometry/odometry.{h,cpp}` -- extraction of the pure integrator (refactor, backward-compatible)

**Reused as-is:**
- `firmware/lib/kinematics/*`, `firmware/lib/pid/*`, `firmware/lib/imu/default_imu.h` (the MPU classes)

**Untouched:** the Arduino/ESP32/Pico/Teensy branches of `encoder.h`/`default_motor.h` and the
existing `platformio.ini`. (The upstream Arduino multi-distro CI and `parse_platformio.py` were
later removed -- this repo does not maintain the upstream legacy.)
