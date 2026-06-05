// Native STM32 quadrature encoder over a TIM in encoder mode. Satisfies the
// portable Encoder API (ctor + getRPM/read/write) so the control loop is shared
// with the Arduino firmware. The leading ctor int (pin1) is a LOGICAL encoder id
// indexing the board's ENC_DESCRIPTORS; pin2 is ignored; a negative id / cpr<=0
// marks an unused wheel (all methods become no-ops). All decision math lives in
// the host-tested EncoderMath; this class only does HAL register work.
#ifndef STM32_ENCODER_H
#define STM32_ENCODER_H

#include "stm32f4xx_hal.h"
#include "encoder_math.h"

class Encoder
{
public:
    Encoder(int pin1, int pin2, int counts_per_rev, bool invert = false);
    void    init();             // HAL TIM/GPIO bring-up; call after HAL_Init (clocks valid)
    float   getRPM();           // advances the math and returns RPM (samples micros internally)
    float   getRPM(uint32_t now_us);  // same, with a caller-supplied timestamp (shared across wheels)
    int32_t read();             // accumulated signed position (as of the last getRPM)
    void    write(int32_t p);   // re-baseline position + counter

private:
    int               id_;
    int               cpr_;
    bool              invert_;
    bool              used_;
    EncoderMath       math_;
    TIM_HandleTypeDef htim_;
};

#endif // STM32_ENCODER_H
