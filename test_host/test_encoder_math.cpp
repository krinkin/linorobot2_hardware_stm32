#include "doctest.h"
#include "encoder_math.h"

// EncoderMath: pure quadrature delta/RPM math (firmware_stm32/Inc/encoder_math.h).

TEST_CASE("EncoderMath: first call seeds and returns 0 (no spurious RPM)") {
    EncoderMath e(1000, /*is32*/false, /*invert*/false);
    CHECK(e.update(54321, 5000) == doctest::Approx(0.0f));
    CHECK(e.position() == 0);
}

TEST_CASE("EncoderMath: RPM = (delta/cpr)/dt_minutes") {
    EncoderMath e(1000, false, false);
    e.update(0, 0);                                  // seed
    // 500 counts in 20 ms: (500/1000) rev / (20000us/60e6) min = 0.5/3.333e-4 = 1500 RPM
    CHECK(e.update(500, 20000) == doctest::Approx(1500.0f));
    CHECK(e.position() == 500);
}

TEST_CASE("EncoderMath: 16-bit counter wraps up correctly (0xFFFE -> 0x0002 = +4)") {
    EncoderMath e(1000, false, false);
    e.update(0xFFFE, 0);
    e.update(0x0002, 1000);
    CHECK(e.position() == 4);
}

TEST_CASE("EncoderMath: 16-bit counter wraps down correctly (0x0002 -> 0xFFFE = -4)") {
    EncoderMath e(1000, false, false);
    e.update(0x0002, 0);
    e.update(0xFFFE, 1000);
    CHECK(e.position() == -4);
}

TEST_CASE("EncoderMath: 32-bit counter delta across the 32-bit boundary") {
    EncoderMath e(1000, /*is32*/true, false);
    e.update(0xFFFFFFF0u, 0);
    e.update(0x00000010u, 1000);
    CHECK(e.position() == 0x20);
}

TEST_CASE("EncoderMath: invert flips the sign of the delta") {
    EncoderMath e(1000, false, /*invert*/true);
    e.update(0, 0);
    e.update(100, 1000);
    CHECK(e.position() == -100);
}

TEST_CASE("EncoderMath: cpr <= 0 marks an unused encoder (always 0)") {
    EncoderMath e(-1, false, false);
    CHECK(e.update(500, 20000) == doctest::Approx(0.0f));
    CHECK(e.update(9999, 40000) == doctest::Approx(0.0f));
    CHECK(e.position() == 0);
}

TEST_CASE("EncoderMath: dt == 0 returns 0 RPM but still accumulates position") {
    EncoderMath e(1000, false, false);
    e.update(100, 1000);                       // seed
    CHECK(e.update(200, 1000) == doctest::Approx(0.0f));   // same timestamp
    CHECK(e.position() == 100);
}

TEST_CASE("EncoderMath: position stays monotonic across repeated wraps") {
    EncoderMath e(1000, false, false);
    e.update(0, 0);
    uint16_t cnt = 0;
    uint32_t t = 0;
    for (int i = 0; i < 10; ++i) { cnt += 10000; t += 1000; e.update(cnt, t); }  // wraps a few times
    CHECK(e.position() == 100000);
}
