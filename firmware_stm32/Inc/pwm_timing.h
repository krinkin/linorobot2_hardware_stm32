// Pure, HAL-free PWM timer-divisor math for the native STM32 port. Host-testable.
//
// A timer driving PWM at PWM_BITS resolution uses ARR = 2^bits - 1 (so a duty in
// [0, ARR] maps to 0..100%). The prescaler is chosen so
//     f_pwm = timer_clk / ((PSC+1) * (ARR+1))  ~= requested freq.
// If the requested freq is too high to reach at the given resolution/clock, PSC
// clamps to 0 (max achievable freq) -- the formula is the contract and self-corrects
// when the clock tree changes (e.g. PLL). PSC saturates at 0xFFFF.
#ifndef PWM_TIMING_H
#define PWM_TIMING_H

#include <stdint.h>
#include <math.h>

struct PwmTiming { uint32_t psc; uint32_t arr; };

inline PwmTiming pwm_timing(uint32_t clk_hz, float freq, int bits)
{
    if (bits < 1)  bits = 1;     // avoid UB shift on negative; floor at a 1-bit timer
    if (bits > 32) bits = 32;
    uint32_t arr = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    PwmTiming t;
    t.arr = arr;
    if (freq <= 0.0f) { t.psc = 0; return t; }
    double ticks = (double)clk_hz / ((double)freq * ((double)arr + 1.0));
    // Clamp in double BEFORE narrowing: on the 32-bit-long target, lround() of a value
    // > 2^31 would be out-of-range/UB. Bounding ticks to [1, 65536] keeps psc in [0,0xFFFF].
    if (ticks < 1.0)     ticks = 1.0;
    if (ticks > 65536.0) ticks = 65536.0;
    t.psc = (uint32_t)(lround(ticks) - 1);
    return t;
}

// Clamp a (possibly negative) duty request into [0, arr] for a CCR write.
inline uint32_t pwm_clamp(int duty, uint32_t arr)
{
    if (duty < 0) duty = 0;
    return ((uint32_t)duty > arr) ? arr : (uint32_t)duty;
}

#endif // PWM_TIMING_H
