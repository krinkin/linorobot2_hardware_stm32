// Peripheral clock-enable dispatch for the native STM32 drivers. Kept out of the
// header so the __HAL_RCC_*_CLK_ENABLE macros (with their read-back barriers) are
// emitted once. Covers the TIM/GPIO instances the F446RE descriptor tables use;
// extend the switches when a new board's tables reference more peripherals.
#include "lino_hal.h"

void lino_tim_clk_enable(TIM_TypeDef* tim)
{
    if      (tim == TIM1) __HAL_RCC_TIM1_CLK_ENABLE();
    else if (tim == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
    else if (tim == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
    else if (tim == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
    else if (tim == TIM5) __HAL_RCC_TIM5_CLK_ENABLE();
    else if (tim == TIM8) __HAL_RCC_TIM8_CLK_ENABLE();
}

void lino_gpio_clk_enable(GPIO_TypeDef* port)
{
    if      (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
}

// A timer's kernel clock. On STM32F4 the APB-domain timer clock is 2x PCLKx whenever the
// APB prescaler != 1 (RM0390 6.2), so we cannot use a single board-wide constant. Derive it
// per timer from the live clock tree -> correct under HSI-16 today and under any future PLL.
uint32_t lino_tim_clk_hz(TIM_TypeDef* tim)
{
    bool apb1 = (tim == TIM2 || tim == TIM3  || tim == TIM4  || tim == TIM5 ||
                 tim == TIM6 || tim == TIM7  || tim == TIM12 || tim == TIM13 || tim == TIM14);
    RCC_ClkInitTypeDef clk; uint32_t flash_latency;
    HAL_RCC_GetClockConfig(&clk, &flash_latency);
    uint32_t pclk = apb1 ? HAL_RCC_GetPCLK1Freq() : HAL_RCC_GetPCLK2Freq();
    uint32_t div  = apb1 ? clk.APB1CLKDivider : clk.APB2CLKDivider;
    return (div == RCC_HCLK_DIV1) ? pclk : pclk * 2u;
}

void lino_i2c_clk_enable(I2C_TypeDef* i2c)
{
    if      (i2c == I2C1) __HAL_RCC_I2C1_CLK_ENABLE();
    else if (i2c == I2C2) __HAL_RCC_I2C2_CLK_ENABLE();
    else if (i2c == I2C3) __HAL_RCC_I2C3_CLK_ENABLE();
}

// delay() shim for the reused Arduino-style libs (imu_interface.h::calibrateGyro).
// Declared in Arduino.h; defined here so the declaration stays HAL/FreeRTOS-free for
// the math TUs (kinematics/pid) that include Arduino.h but never call delay().
void delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// Free-running 1 MHz time base on TIM5 (32-bit) for lino_micros(). No GPIO — internal only.
void lino_time_init(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();
    uint32_t presc = lino_tim_clk_hz(TIM5) / 1000000u;   // -> 1 MHz tick
    TIM5->PSC = (presc > 0u) ? (presc - 1u) : 0u;
    TIM5->ARR = 0xFFFFFFFFu;
    TIM5->EGR = TIM_EGR_UG;          // latch PSC/ARR
    TIM5->CR1 |= TIM_CR1_CEN;        // start counting
}
