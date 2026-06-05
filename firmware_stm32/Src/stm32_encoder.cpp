#include "stm32_encoder.h"
#include "lino_hal.h"
#include "config.h"     // ENC_DESCRIPTORS

Encoder::Encoder(int pin1, int pin2, int counts_per_rev, bool invert)
    : id_(pin1),
      cpr_(counts_per_rev),
      invert_(invert),
      used_(pin1 >= 0 && counts_per_rev > 0),
      math_(used_ ? counts_per_rev : -1, /*is32 (fixed in init)*/false, invert),
      htim_{}
{
    (void)pin2;   // logical-id scheme: the _B arg is unused for selection
}

void Encoder::init()
{
    if (!used_) return;
    const EncoderDesc& d = ENC_DESCRIPTORS[id_];

    // Now that the descriptor is known, fix the counter width for the wrap math.
    math_ = EncoderMath(cpr_, d.is32, invert_);

    lino_tim_clk_enable(d.tim);
    lino_gpio_clk_enable(d.port_a);
    lino_gpio_clk_enable(d.port_b);

    GPIO_InitTypeDef g = {};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = d.af;
    g.Pin = d.pin_a; HAL_GPIO_Init(d.port_a, &g);
    g.Pin = d.pin_b; HAL_GPIO_Init(d.port_b, &g);

    htim_.Instance               = d.tim;
    htim_.Init.Prescaler         = 0;
    htim_.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim_.Init.Period            = d.is32 ? 0xFFFFFFFFu : 0xFFFFu;
    htim_.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef e = {};
    e.EncoderMode  = TIM_ENCODERMODE_TI12;          // count on both TI1 and TI2 edges (x4)
    e.IC1Polarity  = TIM_ICPOLARITY_RISING;
    e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC1Prescaler = TIM_ICPSC_DIV1;
    e.IC1Filter    = 0;
    e.IC2Polarity  = TIM_ICPOLARITY_RISING;
    e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC2Prescaler = TIM_ICPSC_DIV1;
    e.IC2Filter    = 0;

    HAL_TIM_Encoder_Init(&htim_, &e);
    HAL_TIM_Encoder_Start(&htim_, TIM_CHANNEL_ALL);
}

float Encoder::getRPM()
{
    return getRPM(lino_micros());
}

float Encoder::getRPM(uint32_t now_us)
{
    if (!used_) return 0.0f;
    return math_.update((uint32_t)__HAL_TIM_GET_COUNTER(&htim_), now_us);
}

int32_t Encoder::read()
{
    return used_ ? math_.position() : 0;
}

void Encoder::write(int32_t p)
{
    if (!used_) return;
    math_.set_position(p);
    __HAL_TIM_SET_COUNTER(&htim_, (uint32_t)p);
}
