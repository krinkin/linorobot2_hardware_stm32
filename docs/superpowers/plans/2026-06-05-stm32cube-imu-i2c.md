# STM32Cube Port — Plan 6: IMU (MPU6050) over HAL I2C (Phase Ф5)

> **As-built record.** Designed via a 3-lens workflow panel, implemented, and hardened by a
> 4-dimension adversarial review (11 confirmed findings). All tiers green.
>
> **Prerequisite:** Plans 1–5 (host tests, F0 link, Ф2 boot, Ф3 agent round-trip, Ф4 control loop).

**Goal:** the IMU read path for the native port — a register-level **MPU6050** driver over a HAL I2C
adapter, satisfying the reused `IMUInterface`, plus the `FakeIMU`/`FakeMAG` fallbacks. Universal across
any FPU STM32; F446RE first. Publishing `/imu/data_raw` + `/imu/mag` is Plan 7.

**Result (2026-06-05):** Tier A host doctest (imu_math: be16 + LSB→SI scales), Tier B F0/F446 link
(66.8 KB Flash, hard-float, no libstdc++), Ф5 control/IMU smoke — **MPU6050 detected (WHO_AM_I=0x68) +
configured (PWR_MGMT/GYRO/ACCEL_CONFIG) over the REAL HAL I2C path** against a Python mock slave, no fault.

## Architecture (3 layers, as established)
- **Pure core:** `Inc/imu_math.h` — `be16`, `mpu6050_accel/gyro_lsb_to_*`, range-parameterized variants
  (host-tested; ROS/HAL/Arduino-free).
- **HAL adapter:** `Inc/stm32_i2c.{h} + Src/stm32_i2c.cpp` — `I2cBus` over `HAL_I2C_Init` +
  `Mem_Read/Mem_Write/IsDeviceReady` (dev7<<1; strong no-op `HAL_I2C_MspInit`). `lino_hal` gains `I2cDesc`
  + `lino_i2c_clk_enable` + a `delay()` shim (for the reused `calibrateGyro`).
- **Drivers:** `Inc/stm32_imu.h` (`Mpu6050Imu` + verbatim `FakeIMU` + `USE_*_IMU` selector),
  `Inc/stm32_mag.h` (verbatim `FakeMAG` + `USE_*_MAG` selector → FakeMAG). Reuses
  `firmware/lib/imu/{imu_interface,mag_interface}.h` verbatim (ROS msg types from libmicroros).
- **Glue:** `control_loop.cpp` constructs the bus+IMU+MAG in `control_loop_init`, reads `imu->getData()`
  each 50 Hz tick (gated on init success), exposes `g_dbg_imu_ok / i2c_init_ok / imu_read_ok /
  accel_z_milli / gyro_z_milli`.

## Locked decisions
- **Native-from-scratch driver** (no I2Cdevlib vendoring): ~6 MPU6050 registers vs a submodule + an
  I2CDEV-HAL backend + Arduino/Wire/PROGMEM baggage.
- **Scope:** MPU6050 + FakeIMU; MAG = FakeMAG (real magnetometer deferred). MPU9250/GY85/QMI8658 add later
  behind the same selector.
- **F446 I2C:** I2C1 on PB8(SCL)/PB9(SDA) AF4, 100 kHz, open-drain + internal pull-ups (Renode idles high;
  real boards need external 4.7 k — documented). Config-driven `I2cDesc` → universal.
- **Ф5 test:** a Python MPU6050 mock (`Mocks.DummyI2CSlave @ i2c1 0x68`) answers the firmware's real HAL
  I2C transactions. PASS = `imu_ok` (WHO_AM_I + config over real I2C) + no fault. The multi-byte burst
  *data* is a **hardware-only** check: Renode's generic `DummyI2CSlave` returns one byte per master-read,
  so the accel/gyro SI values can't be reproduced in emulation (the driver issues the correct bursts —
  visible in the Renode log; the LSB→SI math is host-tested).

## Adversarial review — 11 findings fixed
- **HIGH:** gyro bias was never removed (the native driver doesn't chip-calibrate, and `getData()` skips
  the software subtraction under `USE_MPU6050_IMU`). Fixed self-contained: `Mpu6050Imu::init()` averages
  40 stationary gyro samples and `readGyroscope()` subtracts that bias (shared `imu_interface.h` untouched).
- **MED:** a faulted I2C bus would inject blocking timeouts into the 50 Hz loop → per-tick read gated on
  init success; `I2C_TIMEOUT_MS` cut to 8 ms. (Full SCL bus-recovery bit-bang = a real-hardware item,
  untestable in Renode — documented below.)
- **MED:** no runtime read-health signal → `readOk()` + `g_dbg_imu_read_ok`.
- **MED:** the "no-fault" PC compare was structurally always-true (uppercase/zero-padded `nm` vs
  lowercase/stripped Renode PC) → numeric compare in `imu_smoke.sh` + `control_smoke.sh`.
- **LOW:** `HAL_I2C_Init` status discarded → `I2cBus::init()` returns bool + `g_dbg_i2c_init_ok`;
  `PWR_MGMT_1` 0x00→0x01 (PLL X-gyro, matching the Arduino path); wake-settle `delay(50)`; check the
  GYRO/ACCEL_CONFIG write returns.

## Deferred (documented)
- **I2C bus recovery** (9-SCL-pulse + STOP bit-bang for an externally stuck SDA) — real-hardware
  robustness, not reproducible in Renode. Add with a hardware watchdog in a later hardware-bring-up pass.
- Plan 7: publish `/imu/data_raw` (+ `/imu/mag` when a real MAG lands) — the read path + health signals
  are in place.

## Reproduce
```bash
source /opt/ros/jazzy/setup.bash
make test-host        # Tier A (imu_math + the Plan 1-5 cores)
make build-fw         # Tier B (clean of libstdc++)
make imu              # Ф5 (MPU6050 over real HAL I2C via the Python mock)
```
