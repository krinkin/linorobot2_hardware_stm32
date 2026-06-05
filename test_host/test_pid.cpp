#include "doctest.h"
#include "pid.h"

TEST_CASE("PID: integral term is deterministic from construction") {
    PID pid(-1000.0f, 1000.0f, /*kp*/0.0f, /*ki*/1.0f, /*kd*/0.0f);
    // error=10, integral=0+10=10 -> ki*integral = 10
    CHECK(pid.compute(10.0f, 0.0f) == doctest::Approx(10.0));
    // integral accumulates across calls: 10 + 5 = 15
    CHECK(pid.compute(5.0f, 0.0f) == doctest::Approx(15.0));
}

TEST_CASE("PID: derivative term is deterministic from construction") {
    PID pid(-1000.0f, 1000.0f, /*kp*/0.0f, /*ki*/0.0f, /*kd*/1.0f);
    // error=10, derivative=10-prev_error(0)=10 -> kd*derivative = 10
    CHECK(pid.compute(10.0f, 0.0f) == doctest::Approx(10.0));
    // second call: prev_error is now 10, so derivative = 5 - 10 = -5
    CHECK(pid.compute(5.0f, 0.0f) == doctest::Approx(-5.0));
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
