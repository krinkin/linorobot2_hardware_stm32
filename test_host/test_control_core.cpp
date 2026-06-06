#include "doctest.h"
#include "control_core.h"
#include <cmath>

// ControlCore: the pure moveBase() step (firmware_stm32/Inc/control_core.h) --
// cmd -> Kinematics::getRPM -> PID -> Motor::spin -> getVelocities -> odom.update.

namespace {
struct FakeMotor : public MotorInterface {
    enum Mode { NONE, FWD, REV, BRAKE } mode = NONE;
    int last_pwm = 0;
    explicit FakeMotor(int invert) : MotorInterface(invert) {}
    void forward(int pwm) override { mode = FWD;   last_pwm = pwm; }
    void reverse(int pwm) override { mode = REV;   last_pwm = pwm; }
    void brake()          override { mode = BRAKE; last_pwm = 0;   }
};

// Differential-drive base: max_rpm = (12/24)*140*0.85 = 59.5; wheel circ = PI*0.1.
Kinematics make_kin() {
    return Kinematics(Kinematics::DIFFERENTIAL_DRIVE, 140, 0.85f,
                      24.0f, 12.0f, 0.1f, 0.3f);
}
} // namespace

TEST_CASE("ControlCore: forward command drives both wheels forward (P-only PID)") {
    Kinematics kin = make_kin();
    OdomIntegrator odom;
    PID p0(-1023, 1023, 1.0f, 0.0f, 0.0f), p1(-1023, 1023, 1.0f, 0.0f, 0.0f);
    FakeMotor m0(0), m1(0);
    ControlCore core(kin, odom, &p0, &p1, nullptr, nullptr, &m0, &m1, nullptr, nullptr);

    const float rpm[4] = {0, 0, 0, 0};
    core.step(/*x*/0.2f, /*y*/0, /*wz*/0, rpm, /*dt*/0.02f);

    // x_rpm = 0.2*60/(PI*0.1) = 38.197; P-only PID(38.197, 0) -> ~38 forward.
    CHECK(m0.mode == FakeMotor::FWD);
    CHECK(m1.mode == FakeMotor::FWD);
    CHECK(m0.last_pwm == 38);
    CHECK(m1.last_pwm == 38);
}

TEST_CASE("ControlCore: zero command + zero motion -> brake (deadman-safe)") {
    Kinematics kin = make_kin();
    OdomIntegrator odom;
    PID p0(-1023, 1023, 1.0f, 0.0f, 0.0f), p1(-1023, 1023, 1.0f, 0.0f, 0.0f);
    FakeMotor m0(0), m1(0);
    ControlCore core(kin, odom, &p0, &p1, nullptr, nullptr, &m0, &m1, nullptr, nullptr);

    const float rpm[4] = {0, 0, 0, 0};
    core.step(0, 0, 0, rpm, 0.02f);

    CHECK(m0.mode == FakeMotor::BRAKE);
    CHECK(m1.mode == FakeMotor::BRAKE);
}

TEST_CASE("ControlCore: odometry integrates measured wheel velocity") {
    Kinematics kin = make_kin();
    OdomIntegrator odom;
    PID p0(-1023, 1023, 1.0f, 0.0f, 0.0f), p1(-1023, 1023, 1.0f, 0.0f, 0.0f);
    FakeMotor m0(0), m1(0);
    ControlCore core(kin, odom, &p0, &p1, nullptr, nullptr, &m0, &m1, nullptr, nullptr);

    // Both wheels measured at 60 RPM -> linear_x = 1.0 rps * (PI*0.1) m = 0.31416 m/s, wz = 0.
    const float rpm[4] = {60, 60, 0, 0};
    const float circ = (float)(M_PI * 0.1);
    for (int i = 0; i < 5; ++i) core.step(0, 0, 0, rpm, 0.02f);

    CHECK(odom.x == doctest::Approx(circ * 1.0f * 0.02f * 5));   // 5 steps of constant velocity
    CHECK(odom.y == doctest::Approx(0.0f));
    CHECK(odom.heading == doctest::Approx(0.0f));
}
