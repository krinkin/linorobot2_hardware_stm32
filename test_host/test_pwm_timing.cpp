#include "doctest.h"
#include "pwm_timing.h"

// pwm_timing: pure PWM divisor math (firmware_stm32/Inc/pwm_timing.h).

TEST_CASE("pwm_timing: ARR = 2^bits - 1") {
    CHECK(pwm_timing(16000000, 1000, 8).arr  == 255u);
    CHECK(pwm_timing(16000000, 1000, 9).arr  == 511u);
    CHECK(pwm_timing(16000000, 1000, 10).arr == 1023u);
}

TEST_CASE("pwm_timing: 20kHz @ 10-bit is unreachable at 16MHz (PSC clamps to 0)") {
    PwmTiming t = pwm_timing(16000000, 20000, 10);
    CHECK(t.arr == 1023u);
    CHECK(t.psc == 0u);
    // Documented finding: achievable freq is ~15.6kHz, NOT the requested 20kHz.
    double achieved = 16000000.0 / ((t.psc + 1.0) * (t.arr + 1.0));
    CHECK(achieved == doctest::Approx(15625.0));
    CHECK(achieved < 20000.0);
}

TEST_CASE("pwm_timing: a reachable frequency yields the expected prescaler") {
    PwmTiming t = pwm_timing(16000000, 1000, 10);   // 16e6/(1000*1024) = 15.625 -> psc=15
    CHECK(t.psc == 15u);
    double achieved = 16000000.0 / ((t.psc + 1.0) * (t.arr + 1.0));
    CHECK(achieved == doctest::Approx(976.5625));
}

TEST_CASE("pwm_timing: PSC saturates at 0xFFFF, and freq<=0 is safe") {
    CHECK(pwm_timing(16000000, 0.01f, 10).psc == 0xFFFFu);   // would need a huge divisor
    CHECK(pwm_timing(16000000, 0.0f, 10).psc == 0u);         // guarded
}

TEST_CASE("pwm_timing: bits is clamped to [1,32] (no UB shift, no degenerate arr)") {
    CHECK(pwm_timing(16000000, 1000, 1).arr  == 1u);
    CHECK(pwm_timing(16000000, 1000, 31).arr == 0x7FFFFFFFu);
    CHECK(pwm_timing(16000000, 1000, 32).arr == 0xFFFFFFFFu);
    CHECK(pwm_timing(16000000, 1000, 0).arr  == 1u);   // clamped up from 0
    CHECK(pwm_timing(16000000, 1000, -4).arr == 1u);   // clamped up from negative (no UB)
}

TEST_CASE("pwm_timing: sub-microhertz freq saturates PSC without 32-bit-long overflow") {
    // ticks here exceeds 2^31; clamping in double keeps psc at 0xFFFF on a 32-bit-long target.
    CHECK(pwm_timing(180000000u, 1e-6f, 10).psc == 0xFFFFu);
}

TEST_CASE("pwm_clamp: bounds a duty request into [0, arr]") {
    CHECK(pwm_clamp(-5, 1023) == 0u);
    CHECK(pwm_clamp(0, 1023) == 0u);
    CHECK(pwm_clamp(500, 1023) == 500u);
    CHECK(pwm_clamp(2000, 1023) == 1023u);
    CHECK(pwm_clamp(1023, 1023) == 1023u);
}
