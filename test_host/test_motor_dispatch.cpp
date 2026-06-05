#include "doctest.h"
#include "motor_interface.h"

// MotorInterface::spin() sign/invert dispatch (firmware_stm32/Inc/motor_interface.h).

namespace {
struct FakeMotor : public MotorInterface {
    enum Mode { NONE, FWD, REV, BRAKE } mode = NONE;
    int last_pwm = 0;
    explicit FakeMotor(int invert) : MotorInterface(invert) {}
    void forward(int pwm) override { mode = FWD;   last_pwm = pwm; }
    void reverse(int pwm) override { mode = REV;   last_pwm = pwm; }
    void brake()          override { mode = BRAKE; last_pwm = 0;   }
};
}

TEST_CASE("spin: positive pwm -> forward, negative -> reverse, zero -> brake") {
    FakeMotor m(/*invert*/0);
    m.spin(100);  CHECK(m.mode == FakeMotor::FWD);   CHECK(m.last_pwm == 100);
    m.spin(-100); CHECK(m.mode == FakeMotor::REV);   CHECK(m.last_pwm == -100);
    m.spin(0);    CHECK(m.mode == FakeMotor::BRAKE);
}

TEST_CASE("spin: invert flips the commanded direction") {
    FakeMotor m(/*invert*/1);
    m.spin(100);  CHECK(m.mode == FakeMotor::REV);   CHECK(m.last_pwm == -100);
    m.spin(-100); CHECK(m.mode == FakeMotor::FWD);   CHECK(m.last_pwm == 100);
    m.spin(0);    CHECK(m.mode == FakeMotor::BRAKE);
}
