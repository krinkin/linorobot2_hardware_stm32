# STM32Cube Port -- Plan 5: Encoder (TIM) + PWM Motor + Control Loop (Phase F4)

> **As-built record.** Plan 5 was designed (3-lens panel -> synthesized spec), implemented, and
> adversarially reviewed (5 dimensions, 13 confirmed findings fixed) on 2026-06-05. All tiers green.
>
> **Prerequisite:** Plans 1-4 done (host tests, F0 link, F2 boot, F3 agent round-trip).

**Goal:** native HAL drivers for the **encoder (STM32 TIM encoder mode)** and **motor (TIM PWM + GPIO
direction)**, plus the **50 Hz control loop** (`/cmd_vel`-style cmd -> `Kinematics::getRPM` -> per-wheel
`PID` over encoder RPM -> `Motor::spin` -> `Kinematics::getVelocities` -> `OdomIntegrator`), reusing the
portable, host-tested math verbatim. Universal across any FPU STM32; F446RE is the first board.

**Result (2026-06-05):** all tiers pass -- Tier A host doctest (35 cases), Tier B F0/F446 link (62.5 KB
Flash / ~73.6 KB RAM, hard-float, **no libstdc++ pulled in**), F2 boot smoke (firmware transmits the
ping), F4 control smoke (scheduler + 50 Hz loop run; an **injected encoder count moves odometry** with
both encoder timers frozen -- a load-bearing end-to-end check).

---

## Architecture (3 layers -- testability boundary)

1. **Pure cores (host-tested, zero HAL):**
   - `Inc/encoder_math.h` -- 16/32-bit counter wrap + RPM/position math.
   - `Inc/pwm_timing.h` -- PSC/ARR from (clk, freq, bits) + duty clamp.
   - `Inc/control_core.h` -- the moveBase step over `Kinematics`/`PID`/`OdomIntegrator`/`MotorInterface`.
   - `Inc/motor_interface.h` -- verbatim copy of the Arduino `MotorInterface` (spin/invert/sign).
2. **Thin HAL adapters (register pokes only):**
   - `Inc/stm32_encoder.{h} + Src/stm32_encoder.cpp` -- TIM encoder-mode init; `getRPM/read/write`.
   - `Inc/stm32_motor.{h} + Src/stm32_motor.cpp` -- `Generic2` (1 PWM ch + 2 dir GPIOs); `#define Motor`.
   - `Inc/lino_hal.{h} + Src/lino_hal.cpp` -- descriptor struct types, per-timer clock derivation,
     clock-enable dispatch, **1 MHz TIM5 microsecond time base**.
3. **App glue:**
   - `Src/control_loop.cpp` (+ `Inc/control_loop.h` extern "C" surface) -- owns the objects, runs the tick.
   - `Src/cxx_runtime.cpp` -- `operator new/delete` (->malloc) + `__cxa_pure_virtual` (no libstdc++).
   - `Src/sys_freertos.c` -- `__malloc_lock/unlock` (scheduler-suspend) + a **bounded `_sbrk`**.
   - `Src/main.c` -- a dedicated `control_task` @ 50 Hz, independent of the micro-ROS agent.

## Locked design decisions

- **C/C++ boundary:** `main.c` stays C; all drivers + control are C++ behind a 4-function `extern "C"`
  surface. No C++ type crosses into `main.c`.
- **Control on its own FreeRTOS task** (not an rclc timer): so the base is governed and the deadman
  brakes even when micro-ROS is disconnected -- and so F4 is testable without the agent.
- **Logical-id config:** the portable ctors' leading ints are reinterpreted as ids indexing per-board
  descriptor tables in `config/f446re_config.h`; a negative id = unused wheel. A second board = a new
  config header, zero driver edits.
- **F446RE map:** encoders M1=TIM2(32-bit) PA0/PA1 AF1, M2=TIM3 PA6/PA7 AF2 (M3=TIM4, M4=TIM1 reserved;
  TIM5 is the us base). PWM = TIM8 CH1..4 on PC6..PC9 AF3. Direction GPIOs PC0..PC3. USART2 (PA2/PA3) and
  micro-ROS untouched. DIFFERENTIAL_DRIVE populates only M1/M2 + TIM8 CH1/CH2.
- **Time bases:** `millis()` = FreeRTOS tick (deadman + odom dt, matches the Arduino reference); `micros()`
  = free-running 1 MHz **TIM5** (true us for the encoder RPM dt; works in Renode and on HW, unlike DWT).
- **PWM:** `PSC/ARR` from `pwm_timing()`; per-timer kernel clock via `lino_tim_clk_hz()` (APB1/APB2 + F4
  doubling). At HSI-16, 20 kHz@10-bit is unreachable (~15.6 kHz actual) -- documented; self-corrects at PLL.

## Adversarial review -- 13 findings fixed (highlights)

- **HIGH:** stock nosys `_sbrk` had no stack-collision bound -> OOM corrupts the stack instead of returning
  NULL. Replaced with a bounded `_sbrk` (reserves the top-of-RAM MSP/IRQ stack, fails with ENOMEM).
- **MED:** encoder RPM dt was 1 ms-quantized (`tick*1000`) vs the reference's true us -> added the 1 MHz
  TIM5 us base; one timestamp threaded into all four `getRPM`.
- **MED:** the Renode injection wasn't load-bearing (the generic TIM free-runs) -> the F4 test now freezes
  both encoder timers and asserts odometry moves *because of* the injected count.
- **MED/LOW:** torn cmd/odom tuple across the two tasks -> `control_set_cmd`/`control_loop_tick`/
  `control_get_odom` exchange snapshots under `taskENTER/EXIT_CRITICAL` (correct even though the callers
  arrive in Plan 7).
- **LOW:** two motors re-ran `HAL_TIM_PWM_Init` on shared TIM8 (EGR=UG re-pulse) -> init each timer base
  once; `pwm_timing` now clamps `bits in [1,32]` and the `lround` divisor in double; removed the board-wide
  `TIMER_CLOCK_HZ` (per-timer derivation kills the APB-doubling trap).

## Deferred to Plan 7 (F6, full loop + topics + CI)
- Bridge `/cmd_vel` (-> `control_set_cmd`) and publish `/odom/unfiltered` (<- `control_get_odom`). The
  concurrency-safe snapshot surface is already in place; `control_set_cmd`/`control_get_odom` are currently
  gc-section-dropped (no caller yet) -- Plan 7 adds the callers.
- Implement Generic1/BTS7960/ESC for the native port (their `USE_*` branches `#error` until then).

## Reproduce
```bash
make test-host       # Tier A (incl. encoder_math/pwm_timing/motor-dispatch/control-core)
make build-fw        # Tier B link (asserts clean of libstdc++)
make renode          # F2
make control         # F4 (load-bearing injected-encoder)
```
