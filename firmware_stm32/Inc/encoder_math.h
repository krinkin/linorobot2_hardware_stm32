// Pure, HAL-free quadrature-encoder delta/RPM math for the native STM32 port.
// Feed the raw timer counter (16- or 32-bit, from a TIM in encoder mode) plus a
// microsecond timestamp; get back RPM since the previous call and an accumulated
// signed position. No Arduino, no HAL -> fully host-unit-testable (Tier A).
//
// 16-bit wrap is handled by the int16_t-cast subtraction trick: it yields a
// correctly signed delta across the 0xFFFF<->0x0000 boundary as long as
// |delta| < 32768 per sample. At COUNTS_PER_REV=144000 (x4) and ~140 RPM the
// per-20ms delta is ~6.7k counts — well inside the limit (Nyquist note).
#ifndef ENCODER_MATH_H
#define ENCODER_MATH_H

#include <stdint.h>

class EncoderMath
{
public:
    // counts_per_rev <= 0 marks an UNUSED encoder (all queries return 0/no-op).
    // is32 = true for 32-bit timers (TIM2/TIM5), false for 16-bit (TIM1/3/4...).
    EncoderMath(int counts_per_rev, bool is32, bool invert)
        : cpr_(counts_per_rev), is32_(is32), invert_(invert) {}

    // RPM since the previous call. The first call only seeds state and returns 0
    // (so a stale power-on counter never produces a spurious huge RPM).
    float update(uint32_t cnt, uint32_t now_us)
    {
        if (cpr_ <= 0) return 0.0f;
        if (first_) { first_ = false; prev_cnt_ = cnt; prev_us_ = now_us; return 0.0f; }

        int32_t delta = is32_
            ? (int32_t)(cnt - prev_cnt_)
            : (int32_t)(int16_t)((uint16_t)cnt - (uint16_t)prev_cnt_);   // modular 16-bit wrap
        if (invert_) delta = -delta;

        accum_ += delta;
        uint32_t dt = now_us - prev_us_;        // modular subtraction wraps safely
        prev_cnt_ = cnt;
        prev_us_  = now_us;
        if (dt == 0) return 0.0f;

        double dt_min = (double)dt / 60000000.0;                 // microseconds -> minutes
        return (float)(((double)delta / (double)cpr_) / dt_min); // revolutions / minute
    }

    int32_t position() const { return (int32_t)accum_; }

    // Re-baseline: set the accumulated position and reseed so the next update()
    // measures a delta relative to the counter value at that next call.
    void set_position(int32_t p) { accum_ = p; first_ = true; }

private:
    int      cpr_;
    bool     is32_;
    bool     invert_;
    bool     first_   = true;
    uint32_t prev_cnt_ = 0;
    uint32_t prev_us_  = 0;
    int64_t  accum_    = 0;
};

#endif // ENCODER_MATH_H
