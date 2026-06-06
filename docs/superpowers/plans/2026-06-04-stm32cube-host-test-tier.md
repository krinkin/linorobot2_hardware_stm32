# STM32Cube Port -- Plan 1: Host Test Tier + Odometry Refactor

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stand up a host-native (no MCU, no board) unit-test harness for the portable libraries (`kinematics`, `pid`, `odometry`) and refactor `odometry` so its dead-reckoning math is testable without linking the micro-ROS/`nav_msgs` runtime.

**Architecture:** A standalone `test_host/` project compiled with plain `g++` + a vendored single-header test framework (doctest). The portable math sources are compiled directly against a tiny `Arduino.h` shim (provides `PI`, `constrain`, `fabs`). The ROS-coupled `odometry` integrator is extracted into a header-only, dependency-free `OdomIntegrator`; `Odometry` keeps its public API and delegates to it. This is Tier A of the 3-tier "emulation-first" strategy in `docs/STM32CUBE_PORTING_PLAN.md` section 6 -- the part you can run today with zero hardware.

**Tech Stack:** C++17, `g++`, GNU Make, `curl` (one-time fetch of `doctest.h` v2.4.11, MIT), doctest test framework. No PlatformIO, no STM32 toolchain, no board.

---

## Where this plan sits (full decomposition)

The native STM32Cube port (spec: `docs/STM32CUBE_PORTING_PLAN.md`) is split into a sequence of independently-testable plans. Each later plan is written when its prerequisites/artifacts exist (CubeMX project, pin-map, Renode `.repl`):

1. **Plan 1 -- Host test tier + odometry refactor** <- *this document* (Tier A; spec F1). No toolchain/board needed.
2. **Plan 2 -- STM32 skeleton + libmicroros ABI-smoke** (spec F0). Hand-written F446RE HAL/FreeRTOS project + Makefile (**GUI-free, no CubeMX**) + Docker `microros/micro_ros_static_library_builder:jazzy`; link smoke.
3. **Plan 3 -- Renode harness + boot/FreeRTOS/`rclc_support_init` smoke** (spec F2).
4. **Plan 4 -- micro-ROS UART transport (DMA/IT compile-time switch) + session round-trip in Renode** (spec F3).
5. **Plan 5 -- Encoder (TIM encoder mode) + Motor PWM (TIM) native drivers** (spec F4).
6. **Plan 6 -- I2Cdev HAL backend + MPU6050/9250 IMU path** (spec F5).
7. **Plan 7 -- Full loop in Renode + dedicated CI workflow** (spec F6, CI section 7).

Plan 1 is intentionally first: it is fully self-contained, needs no STM32 toolchain, and gives a fast regression gate for the math that every later phase depends on.

---

## File Structure

| File | Responsibility | Action |
|---|---|---|
| `test_host/Makefile` | Build + run the host test binary | Create |
| `test_host/shim/Arduino.h` | Minimal Arduino API for host (`PI`, `constrain`, `fabs`) | Create |
| `test_host/doctest.h` | Vendored single-header test framework (v2.4.11, MIT) | Create (fetch) |
| `test_host/test_main.cpp` | doctest `main()` entry point (one TU) | Create |
| `test_host/test_kinematics.cpp` | Characterization tests for `Kinematics` | Create |
| `test_host/test_pid.cpp` | Tests for `PID` (incl. deterministic-init) | Create |
| `test_host/test_odom_integrator.cpp` | Tests for the new `OdomIntegrator` | Create |
| `firmware/lib/odom_integrator/odom_integrator.h` | Pure, header-only dead-reckoning integrator + `euler_to_quat` | Create |
| `firmware/lib/pid/pid.cpp` | Initialize `integral_/derivative_/prev_error_` | Modify |
| `firmware/lib/odometry/odometry.h` | Delegate state to `OdomIntegrator` | Modify |
| `firmware/lib/odometry/odometry.cpp` | Use `OdomIntegrator` for the math | Modify |

`.gitignore`: the built binary `test_host/run_tests` should not be committed (Task 1 adds it).

---

### Task 1: Host test harness scaffold

**Files:**
- Create: `test_host/shim/Arduino.h`
- Create: `test_host/doctest.h` (fetched)
- Create: `test_host/test_main.cpp`
- Create: `test_host/Makefile`
- Create: `test_host/.gitignore`

- [ ] **Step 1: Fetch the vendored test framework (pinned, MIT)**

Run:
```bash
mkdir -p test_host/shim
curl -fsSL https://raw.githubusercontent.com/doctest/doctest/v2.4.11/doctest/doctest.h -o test_host/doctest.h
```
Expected: `test_host/doctest.h` exists, ~7800 lines. Verify:
```bash
head -n 5 test_host/doctest.h
```
Expected: header comment mentioning `doctest.h - the lightest feature-rich C++ ... testing framework`.

- [ ] **Step 2: Create the Arduino shim**

Create `test_host/shim/Arduino.h`:
```cpp
// Host-only shim: just enough of the Arduino API for the portable libs to
// compile under g++. NOT used in any firmware build.
#ifndef HOST_TEST_ARDUINO_SHIM_H
#define HOST_TEST_ARDUINO_SHIM_H

#include <math.h>   // fabs, cos, sin
#include <stdint.h>

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

// Arduino's constrain is a macro and works with mixed float/double args
// (pid.cpp calls constrain(double, float, float)); keep it a macro.
#ifndef constrain
#define constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

#endif // HOST_TEST_ARDUINO_SHIM_H
```

- [ ] **Step 3: Create the doctest main entry point + a sanity test**

Create `test_host/test_main.cpp`:
```cpp
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

TEST_CASE("sanity: the host test harness builds and runs") {
    CHECK(1 + 1 == 2);
}
```

- [ ] **Step 4: Create the Makefile**

Create `test_host/Makefile`:
```make
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O0 -g
LIBDIR   := ../firmware/lib
INCLUDES := -Ishim \
            -I$(LIBDIR)/kinematics \
            -I$(LIBDIR)/pid \
            -I$(LIBDIR)/odom_integrator

# Test translation units (added as tasks land)
TESTS := test_main.cpp \
         test_kinematics.cpp \
         test_pid.cpp \
         test_odom_integrator.cpp

# Portable sources under test (header-only libs need no entry here)
SRCS  := $(LIBDIR)/kinematics/kinematics.cpp \
         $(LIBDIR)/pid/pid.cpp

run_tests: $(TESTS) $(SRCS) doctest.h shim/Arduino.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TESTS) $(SRCS) -o run_tests

.PHONY: test clean
test: run_tests
	./run_tests
clean:
	rm -f run_tests
```

> Note: the Makefile already lists `test_kinematics.cpp`, `test_pid.cpp`, `test_odom_integrator.cpp` (created in later tasks). Until those exist the build fails -- that is expected and is exactly the "failing test" for each subsequent task. To run ONLY the sanity check in this task, temporarily build with just `test_main.cpp` (next step).

- [ ] **Step 5: Verify the harness compiles and the sanity test passes**

Run (explicitly only the main TU for this task):
```bash
cd test_host && g++ -std=c++17 -Ishim test_main.cpp -o run_tests && ./run_tests
```
Expected output ends with:
```
[doctest] test cases:  1 |  1 passed | 0 failed | 0 skipped
[doctest] assertions:  1 |  1 passed | 0 failed
[doctest] Status: SUCCESS!
```
Exit code 0.

- [ ] **Step 6: Add .gitignore and commit the scaffold**

Create `test_host/.gitignore`:
```
run_tests
```

Run:
```bash
git add test_host/Makefile test_host/shim/Arduino.h test_host/doctest.h test_host/test_main.cpp test_host/.gitignore
git commit -m "test(host): add host unit-test harness (doctest + Arduino shim)"
```

---

### Task 2: Characterization tests for `Kinematics`

`Kinematics` (`firmware/lib/kinematics/kinematics.{h,cpp}`) already works; these tests pin its behavior so the later port cannot silently change it. With `motor_max_rpm=100`, `max_rpm_ratio=1.0`, `operating_voltage=12`, `power_max_voltage=12`, `wheel_diameter=0.1`, `wheels_y_distance=0.3`: `max_rpm_ = 100`, `wheel_circumference_ = PI*0.1 ~= 0.3141593`.

**Files:**
- Create: `test_host/test_kinematics.cpp`

- [ ] **Step 1: Write the characterization tests**

Create `test_host/test_kinematics.cpp`:
```cpp
#include "doctest.h"
#include "kinematics.h"

static Kinematics make_diff() {
    // base, motor_max_rpm, max_rpm_ratio, operating_voltage,
    // power_max_voltage, wheel_diameter, wheels_y_distance
    return Kinematics(Kinematics::DIFFERENTIAL_DRIVE, 100, 1.0f,
                      12.0f, 12.0f, 0.1f, 0.3f);
}

TEST_CASE("Kinematics: max RPM derives from voltage ratio and ratio factor") {
    Kinematics kin = make_diff();
    CHECK(kin.getMaxRPM() == doctest::Approx(100.0));
}

TEST_CASE("Kinematics: driving straight gives equal left/right wheel RPM") {
    Kinematics kin = make_diff();
    Kinematics::rpm r = kin.getRPM(0.1f, 0.0f, 0.0f); // 0.1 m/s forward
    // 0.1 m/s -> 6 m/min / 0.3141593 m = 19.0986 rpm
    CHECK(r.motor1 == doctest::Approx(19.0986).epsilon(0.001));
    CHECK(r.motor2 == doctest::Approx(r.motor1));
}

TEST_CASE("Kinematics: excessive command saturates to max RPM") {
    Kinematics kin = make_diff();
    Kinematics::rpm r = kin.getRPM(10.0f, 0.0f, 0.0f); // way over max
    CHECK(r.motor1 == doctest::Approx(100.0));
    CHECK(r.motor2 == doctest::Approx(100.0));
}

TEST_CASE("Kinematics: pure rotation is antisymmetric across the axle") {
    Kinematics kin = make_diff();
    Kinematics::rpm r = kin.getRPM(0.0f, 0.0f, 1.0f); // 1 rad/s
    // tangential = 1*(0.3/2)=0.15 m/s -> 9 m/min / 0.3141593 = 28.6479 rpm
    CHECK(r.motor2 == doctest::Approx(28.6479).epsilon(0.001));
    CHECK(r.motor1 == doctest::Approx(-r.motor2));
}

TEST_CASE("Kinematics: getVelocities round-trips a straight-line command") {
    Kinematics kin = make_diff();
    Kinematics::velocities v = kin.getVelocities(19.0986f, 19.0986f, 0.0f, 0.0f);
    CHECK(v.linear_x == doctest::Approx(0.1).epsilon(0.001));
    CHECK(v.angular_z == doctest::Approx(0.0).epsilon(0.001));
}
```

- [ ] **Step 2: Run the tests (characterization -- expected PASS)**

Run:
```bash
cd test_host && g++ -std=c++17 -Ishim -I../firmware/lib/kinematics \
  test_main.cpp test_kinematics.cpp ../firmware/lib/kinematics/kinematics.cpp \
  -o run_tests && ./run_tests
```
Expected: `Status: SUCCESS!`, all assertions passed. (These pin existing behavior, so they pass immediately.)

- [ ] **Step 3: Commit**

```bash
git add test_host/test_kinematics.cpp
git commit -m "test(host): characterize Kinematics RPM/velocity behavior"
```

---

### Task 3: `PID` deterministic-initialization (real TDD)

`PID::compute` accumulates `integral_` and reads `prev_error_`, but the constructor never initializes them (`firmware/lib/pid/pid.cpp:18-25`). On the host a stack-allocated `PID` therefore yields indeterminate output. Fix: initialize them -- this makes the module correct and deterministically testable.

**Files:**
- Create: `test_host/test_pid.cpp`
- Modify: `firmware/lib/pid/pid.cpp:18-25`

- [ ] **Step 1: Write the failing test (deterministic integral/derivative)**

Create `test_host/test_pid.cpp`:
```cpp
#include "doctest.h"
#include "pid.h"

TEST_CASE("PID: integral term is deterministic from construction") {
    PID pid(-1000.0f, 1000.0f, /*kp*/0.0f, /*ki*/1.0f, /*kd*/0.0f);
    // error=10, integral=0+10=10 -> ki*integral = 10
    CHECK(pid.compute(10.0f, 0.0f) == doctest::Approx(10.0));
}

TEST_CASE("PID: derivative term is deterministic from construction") {
    PID pid(-1000.0f, 1000.0f, /*kp*/0.0f, /*ki*/0.0f, /*kd*/1.0f);
    // error=10, derivative=10-prev_error(0)=10 -> kd*derivative = 10
    CHECK(pid.compute(10.0f, 0.0f) == doctest::Approx(10.0));
}

TEST_CASE("PID: proportional term saturates to [min,max]") {
    PID pid(-5.0f, 5.0f, /*kp*/1.0f, /*ki*/0.0f, /*kd*/0.0f);
    CHECK(pid.compute(100.0f, 0.0f) == doctest::Approx(5.0));
    CHECK(pid.compute(-100.0f, 0.0f) == doctest::Approx(-5.0));
}

TEST_CASE("PID: integral resets when setpoint and error are both zero") {
    PID pid(-1000.0f, 1000.0f, /*kp*/0.0f, /*ki*/1.0f, /*kd*/0.0f);
    CHECK(pid.compute(10.0f, 0.0f) == doctest::Approx(10.0)); // integral -> 10
    CHECK(pid.compute(0.0f, 0.0f) == doctest::Approx(0.0));   // reset -> 0
}
```

- [ ] **Step 2: Run to verify it fails (or is non-deterministic)**

Run:
```bash
cd test_host && g++ -std=c++17 -Ishim -I../firmware/lib/pid \
  test_main.cpp test_pid.cpp ../firmware/lib/pid/pid.cpp \
  -o run_tests && ./run_tests
```
Expected: FAIL -- the first two cases fail (or vary run-to-run) because `integral_`/`prev_error_` are uninitialized. Example:
```
ERROR: test_pid.cpp(8): CHECK( pid.compute(10.0f, 0.0f) == doctest::Approx(10.0) ) is NOT correct!
  values: CHECK( <garbage> == Approx( 10.0 ) )
[doctest] Status: FAILURE!
```

- [ ] **Step 3: Initialize the members in the constructor**

Modify `firmware/lib/pid/pid.cpp` -- replace the constructor (lines 18-25):
```cpp
PID::PID(float min_val, float max_val, float kp, float ki, float kd):
    min_val_(min_val),
    max_val_(max_val),
    kp_(kp),
    ki_(ki),
    kd_(kd),
    integral_(0.0),
    derivative_(0.0),
    prev_error_(0.0)
{
}
```

- [ ] **Step 4: Run to verify it passes**

Run:
```bash
cd test_host && g++ -std=c++17 -Ishim -I../firmware/lib/pid \
  test_main.cpp test_pid.cpp ../firmware/lib/pid/pid.cpp \
  -o run_tests && ./run_tests
```
Expected: `Status: SUCCESS!` -- all 4 PID cases pass.

- [ ] **Step 5: Commit**

```bash
git add test_host/test_pid.cpp firmware/lib/pid/pid.cpp
git commit -m "fix(pid): initialize integral/derivative/prev_error for deterministic output

Adds host tests pinning PID behavior; initializes accumulator state in the
constructor (was indeterminate for non-static instances)."
```

---

### Task 4: Extract `OdomIntegrator` (header-only, dependency-free)

Create the pure integrator that `odometry` will delegate to. It reproduces the existing math in `odometry.cpp:26-44,87-100` exactly (position delta uses the *current* heading, then heading is advanced; quaternion layout is `q[0]=w, q[1]=x, q[2]=y, q[3]=z`).

**Files:**
- Create: `firmware/lib/odom_integrator/odom_integrator.h`
- Create: `test_host/test_odom_integrator.cpp`

- [ ] **Step 1: Write the failing test**

Create `test_host/test_odom_integrator.cpp`:
```cpp
#include "doctest.h"
#include "odom_integrator.h"

TEST_CASE("OdomIntegrator: straight motion advances x only") {
    OdomIntegrator odo;
    odo.update(/*dt*/1.0f, /*vx*/1.0f, /*vy*/0.0f, /*wz*/0.0f);
    CHECK(odo.x == doctest::Approx(1.0));
    CHECK(odo.y == doctest::Approx(0.0));
    CHECK(odo.heading == doctest::Approx(0.0));
}

TEST_CASE("OdomIntegrator: rotate 90deg then drive forward moves along +y") {
    OdomIntegrator odo;
    odo.update(1.0f, 0.0f, 0.0f, 1.5707963f); // +pi/2 rad/s for 1 s
    CHECK(odo.heading == doctest::Approx(1.5707963));
    odo.update(1.0f, 1.0f, 0.0f, 0.0f);       // 1 m/s forward, now facing +y
    CHECK(odo.x == doctest::Approx(0.0).epsilon(0.001));
    CHECK(odo.y == doctest::Approx(1.0).epsilon(0.001));
}

TEST_CASE("OdomIntegrator: euler_to_quat identity") {
    float q[4];
    OdomIntegrator::euler_to_quat(0.0f, 0.0f, 0.0f, q);
    CHECK(q[0] == doctest::Approx(1.0)); // w
    CHECK(q[1] == doctest::Approx(0.0));
    CHECK(q[2] == doctest::Approx(0.0));
    CHECK(q[3] == doctest::Approx(0.0));
}

TEST_CASE("OdomIntegrator: euler_to_quat 180deg yaw") {
    float q[4];
    OdomIntegrator::euler_to_quat(0.0f, 0.0f, 3.1415927f, q);
    CHECK(q[0] == doctest::Approx(0.0).epsilon(0.001)); // w
    CHECK(q[3] == doctest::Approx(1.0).epsilon(0.001)); // z
}
```

- [ ] **Step 2: Run to verify it fails (missing header)**

Run:
```bash
cd test_host && g++ -std=c++17 -Ishim -I../firmware/lib/odom_integrator \
  test_main.cpp test_odom_integrator.cpp -o run_tests && ./run_tests
```
Expected: FAIL -- compile error `fatal error: odom_integrator.h: No such file or directory`.

- [ ] **Step 3: Create the header-only integrator**

Create `firmware/lib/odom_integrator/odom_integrator.h`:
```cpp
// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ODOM_INTEGRATOR_H
#define ODOM_INTEGRATOR_H

#include <cmath>

// Pure dead-reckoning integrator + Euler->quaternion conversion.
// No Arduino, no ROS, no micro-ROS dependencies -> host-unit-testable.
class OdomIntegrator
{
    public:
        float x = 0.0f;
        float y = 0.0f;
        float heading = 0.0f;

        void update(float vel_dt, float linear_vel_x, float linear_vel_y,
                    float angular_vel_z)
        {
            float delta_heading = angular_vel_z * vel_dt;        // radians
            float cos_h = std::cos(heading);
            float sin_h = std::sin(heading);
            float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt; // m
            float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt; // m

            x += delta_x;
            y += delta_y;
            heading += delta_heading;
        }

        // q layout: q[0]=w, q[1]=x, q[2]=y, q[3]=z
        static void euler_to_quat(float roll, float pitch, float yaw, float* q)
        {
            float cy = std::cos(yaw * 0.5f);
            float sy = std::sin(yaw * 0.5f);
            float cp = std::cos(pitch * 0.5f);
            float sp = std::sin(pitch * 0.5f);
            float cr = std::cos(roll * 0.5f);
            float sr = std::sin(roll * 0.5f);

            q[0] = cy * cp * cr + sy * sp * sr;
            q[1] = cy * cp * sr - sy * sp * cr;
            q[2] = sy * cp * sr + cy * sp * cr;
            q[3] = sy * cp * cr - cy * sp * sr;
        }
};

#endif // ODOM_INTEGRATOR_H
```

- [ ] **Step 4: Run to verify it passes**

Run:
```bash
cd test_host && g++ -std=c++17 -Ishim -I../firmware/lib/odom_integrator \
  test_main.cpp test_odom_integrator.cpp -o run_tests && ./run_tests
```
Expected: `Status: SUCCESS!` -- all 4 cases pass.

- [ ] **Step 5: Commit**

```bash
git add firmware/lib/odom_integrator/odom_integrator.h test_host/test_odom_integrator.cpp
git commit -m "feat(odometry): extract pure header-only OdomIntegrator with host tests"
```

---

### Task 5: Make `Odometry` delegate to `OdomIntegrator`

Rewire `Odometry` to use `OdomIntegrator` for all dead-reckoning math, keeping the public API (`update`, `getData`) byte-identical to callers. This is a mechanical delegation -- the firmware (`firmware.ino`) and all existing PlatformIO envs keep compiling unchanged.

**Files:**
- Modify: `firmware/lib/odometry/odometry.h:38-45`
- Modify: `firmware/lib/odometry/odometry.cpp:17-45,87-100`

- [ ] **Step 1: Update the header to hold an `OdomIntegrator`**

In `firmware/lib/odometry/odometry.h`, add the include after the existing includes (after line 22, `#include "config.h"`):
```cpp
#include "odom_integrator.h"
```

Then replace the `private:` section (lines 38-45) with:
```cpp
    private:
        nav_msgs__msg__Odometry odom_msg_;
        OdomIntegrator integrator_;
};
```
(This removes the now-unused `euler_to_quat` declaration and the `x_pos_/y_pos_/heading_` members -- they live in `OdomIntegrator` now.)

- [ ] **Step 2: Update the constructor (drop the removed members)**

In `firmware/lib/odometry/odometry.cpp`, replace the constructor (lines 17-24) with:
```cpp
Odometry::Odometry()
{
    odom_msg_.header.frame_id = micro_ros_string_utilities_set(odom_msg_.header.frame_id, "odom");
    odom_msg_.child_frame_id = micro_ros_string_utilities_set(odom_msg_.child_frame_id, "base_footprint");
}
```

- [ ] **Step 3: Update `update()` to delegate the math**

In `firmware/lib/odometry/odometry.cpp`, replace the body of `update()` up to and including the quaternion fill (lines 26-55) with:
```cpp
void Odometry::update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z)
{
    integrator_.update(vel_dt, linear_vel_x, linear_vel_y, angular_vel_z);

    const float pose_cov[6] = POSE_COV;
    const float twist_cov[6] = TWIST_COV;

    //calculate robot's heading in quaternion angle
    float q[4];
    OdomIntegrator::euler_to_quat(0, 0, integrator_.heading, q);

    //robot's position in x,y, and z
    odom_msg_.pose.pose.position.x = integrator_.x;
    odom_msg_.pose.pose.position.y = integrator_.y;
    odom_msg_.pose.pose.position.z = 0.0;

    //robot's heading in quaternion
    odom_msg_.pose.pose.orientation.x = (double) q[1];
    odom_msg_.pose.pose.orientation.y = (double) q[2];
    odom_msg_.pose.pose.orientation.z = (double) q[3];
    odom_msg_.pose.pose.orientation.w = (double) q[0];
```
(Leave the rest of `update()` -- the covariance and twist assignments, lines 57-79 -- unchanged.)

- [ ] **Step 4: Remove the old private `euler_to_quat` definition**

In `firmware/lib/odometry/odometry.cpp`, delete the entire `Odometry::euler_to_quat` definition (lines 87-100), since it now lives in `OdomIntegrator`.

- [ ] **Step 5: Verify the odom_integrator host tests still pass**

Run:
```bash
cd test_host && g++ -std=c++17 -Ishim -I../firmware/lib/odom_integrator \
  test_main.cpp test_odom_integrator.cpp -o run_tests && ./run_tests
```
Expected: `Status: SUCCESS!` (the math is unchanged; this confirms the extracted integrator the firmware now uses is still correct).

- [ ] **Step 6: (Optional, needs the embedded toolchain) regression-build one existing env**

Only if PlatformIO + a ROS distro are available (`source /opt/ros/jazzy/setup.bash`):
```bash
cd firmware && pio run -e esp32
```
Expected: build SUCCESS -- confirms the `odometry` refactor did not break existing firmware builds. If the toolchain is unavailable, skip; the change is a pure mechanical delegation with no API change.

- [ ] **Step 7: Commit**

```bash
git add firmware/lib/odometry/odometry.h firmware/lib/odometry/odometry.cpp
git commit -m "refactor(odometry): delegate dead-reckoning math to OdomIntegrator

Public API unchanged; the ROS-coupled Odometry now wraps the pure,
host-testable OdomIntegrator. No behavior change."
```

---

### Task 6: Wire the full host-test target and document it

Make `make test` build and run every host test together, and record how to run the tier.

**Files:**
- Modify: `test_host/Makefile` (already lists all TUs -- verify it links)
- Create: `test_host/README.md`

- [ ] **Step 1: Run the full suite via the Makefile**

Run:
```bash
cd test_host && make clean && make test
```
Expected output ends with all suites combined, e.g.:
```
[doctest] test cases: 13 | 13 passed | 0 failed | 0 skipped
[doctest] assertions: ... |  ... passed | 0 failed
[doctest] Status: SUCCESS!
```
Exit code 0. (If linking fails, confirm `test_host/Makefile` `INCLUDES`/`SRCS` match Task 1 Step 4.)

- [ ] **Step 2: Document the tier**

Create `test_host/README.md`:
```markdown
# Host test tier

Native unit tests for the portable libraries (`kinematics`, `pid`, `odom_integrator`).
Runs on the host with no MCU, board, or ROS install -- Tier A of the emulation-first
strategy in `docs/STM32CUBE_PORTING_PLAN.md`.

## Prerequisites
- `g++` (C++17), GNU `make`, `curl` (one-time, to fetch `doctest.h`).

## Run
```bash
cd test_host
make test
```
Expected: `[doctest] Status: SUCCESS!`, exit code 0.

## Layout
- `shim/Arduino.h` -- minimal Arduino API (`PI`, `constrain`, `fabs`) for host builds.
- `doctest.h` -- vendored single-header framework (v2.4.11, MIT).
- `test_*.cpp` -- one TU per library under test; `test_main.cpp` owns `main()`.
```

- [ ] **Step 3: Commit**

```bash
git add test_host/Makefile test_host/README.md
git commit -m "test(host): wire full make test target and document the tier"
```

---

## Self-Review (performed against the spec)

- **Spec coverage (Tier A / F1):** host tests for `kinematics` (Task 2), `pid` (Task 3), and the extracted `odometry` integrator (Tasks 4-5) -- matches spec section 1 Layer 1, section 5.6 (odometry refactor away from `nav_msgs`), section 6 Tier A, and section 8 F1. The other tiers/phases are out of scope for Plan 1 by design (decomposition list above).
- **Placeholder scan:** no TBD/TODO; every code step shows complete code; every run step shows the exact command and expected output. [OK]
- **Type/name consistency:** `OdomIntegrator` public members `x/y/heading` and `static euler_to_quat(roll,pitch,yaw,q)` are defined in Task 4 and used identically in Task 5; `PID` constructor signature matches `pid.h:23`; `Kinematics` ctor/`getRPM`/`getVelocities`/`getMaxRPM` match `kinematics.h`. [OK]
- **Known limitation:** the optional existing-env regression build (Task 5 Step 6) needs the PlatformIO + ROS toolchain, which may be absent in a no-hardware setup; it is explicitly marked optional and the refactor is API-preserving.
