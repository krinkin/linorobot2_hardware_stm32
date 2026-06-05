#include "stm32_motor.h"
#include "lino_hal.h"
#include "pwm_timing.h"
#include "config.h"     // PWM_DESCRIPTORS, DIR_DESCRIPTORS
#include <stdlib.h>     // abs

// Claim a TIM's time base exactly once across all Generic2 instances. Several motors
// can share one timer (e.g. TIM8 CH1/CH2 on DIFFERENTIAL_DRIVE); HAL_TIM_PWM_Init forces
// EGR=UG (a counter reset) on every call, which would re-pulse an already-running shared
// timer. So the FIRST channel to claim a timer does the base init; siblings only configure
// + start their own channel on the shared time base.
namespace {
TIM_TypeDef* s_pwm_base_inited[8] = {0};
bool pwm_base_claim(TIM_TypeDef* t)
{
    for (auto& s : s_pwm_base_inited) {
        if (s == t)  return false;   // already initialised by a sibling channel
        if (s == 0)  { s = t; return true; }
    }
    return false;
}
}

Generic2::Generic2(float pwm_frequency, int pwm_bits, bool invert,
                   int pwm_id, int in_a_id, int in_b_id)
    : MotorInterface(invert),
      freq_(pwm_frequency),
      bits_(pwm_bits),
      pwm_id_(pwm_id), in_a_id_(in_a_id), in_b_id_(in_b_id),
      used_(pwm_id >= 0 && in_a_id >= 0 && in_b_id >= 0),
      arr_((1u << pwm_bits) - 1u),
      channel_(0),
      dir_a_port_(nullptr), dir_a_pin_(0),
      dir_b_port_(nullptr), dir_b_pin_(0),
      htim_{}
{
}

void Generic2::init()
{
    if (!used_) return;
    const PwmDesc&  p = PWM_DESCRIPTORS[pwm_id_];
    const GpioDesc& a = DIR_DESCRIPTORS[in_a_id_];
    const GpioDesc& b = DIR_DESCRIPTORS[in_b_id_];
    channel_    = p.channel;
    dir_a_port_ = a.port; dir_a_pin_ = a.pin;
    dir_b_port_ = b.port; dir_b_pin_ = b.pin;

    lino_tim_clk_enable(p.tim);
    lino_gpio_clk_enable(p.port);
    lino_gpio_clk_enable(a.port);
    lino_gpio_clk_enable(b.port);

    // Time base (shared if several channels live on one timer; identical PSC/ARR).
    PwmTiming t = pwm_timing(lino_tim_clk_hz(p.tim), freq_, bits_);
    arr_ = t.arr;
    htim_.Instance               = p.tim;
    htim_.Init.Prescaler         = t.psc;
    htim_.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim_.Init.Period            = t.arr;
    htim_.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_.Init.RepetitionCounter = 0;
    htim_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (pwm_base_claim(p.tim))   // base init once per physical timer (avoids EGR=UG re-pulse)
        HAL_TIM_PWM_Init(&htim_);

    GPIO_InitTypeDef g = {};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = p.af;
    g.Pin       = p.pin;
    HAL_GPIO_Init(p.port, &g);

    TIM_OC_InitTypeDef oc = {};
    oc.OCMode      = TIM_OCMODE_PWM1;
    oc.Pulse       = 0;
    oc.OCPolarity  = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode  = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState= TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim_, &oc, channel_);
    HAL_TIM_PWM_Start(&htim_, channel_);
    __HAL_TIM_MOE_ENABLE(&htim_);     // advanced-timer main output enable (TIM1/TIM8)

    GPIO_InitTypeDef d = {};
    d.Mode  = GPIO_MODE_OUTPUT_PP;
    d.Pull  = GPIO_NOPULL;
    d.Speed = GPIO_SPEED_FREQ_LOW;
    d.Pin = a.pin; HAL_GPIO_Init(a.port, &d);
    d.Pin = b.pin; HAL_GPIO_Init(b.port, &d);
    HAL_GPIO_WritePin(a.port, a.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(b.port, b.pin, GPIO_PIN_RESET);
}

void Generic2::forward(int pwm)
{
    if (!used_) return;
    HAL_GPIO_WritePin(dir_a_port_, dir_a_pin_, GPIO_PIN_SET);
    HAL_GPIO_WritePin(dir_b_port_, dir_b_pin_, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim_, channel_, pwm_clamp(pwm, arr_));
}

void Generic2::reverse(int pwm)
{
    if (!used_) return;
    HAL_GPIO_WritePin(dir_a_port_, dir_a_pin_, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(dir_b_port_, dir_b_pin_, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim_, channel_, pwm_clamp(abs(pwm), arr_));
}

void Generic2::brake()
{
    if (!used_) return;
    __HAL_TIM_SET_COMPARE(&htim_, channel_, 0);
#ifdef USE_SHORT_BRAKE
    HAL_GPIO_WritePin(dir_a_port_, dir_a_pin_, GPIO_PIN_SET);
    HAL_GPIO_WritePin(dir_b_port_, dir_b_pin_, GPIO_PIN_SET);
#endif
}
