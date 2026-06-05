// Shared HAL plumbing for the native STM32 encoder/motor drivers: the per-board
// descriptor struct types, peripheral clock-enable helpers, and a FreeRTOS-tick
// time base. Board-specific CONSTANTS live in config/<board>_config.h, not here.
// Firmware-only (pulls HAL + FreeRTOS); the pure cores never include this.
#ifndef LINO_HAL_H
#define LINO_HAL_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

// One quadrature encoder bound to a TIM in encoder mode (CH1/CH2 on AF pins).
struct EncoderDesc {
    TIM_TypeDef*  tim;
    uint32_t      af;
    GPIO_TypeDef* port_a; uint16_t pin_a;
    GPIO_TypeDef* port_b; uint16_t pin_b;
    bool          is32;          // 32-bit counter (TIM2/TIM5) vs 16-bit
};

// One PWM output: a TIM channel on an AF pin.
struct PwmDesc {
    TIM_TypeDef*  tim;
    uint32_t      af;
    GPIO_TypeDef* port; uint16_t pin;
    uint32_t      channel;       // TIM_CHANNEL_1..4
};

// One direction GPIO (push-pull output).
struct GpioDesc {
    GPIO_TypeDef* port; uint16_t pin;
};

// Time bases:
//   lino_millis() = the FreeRTOS tick (configTICK_RATE_HZ = 1000 => 1 ms) — used for the
//     deadman and odometry dt (matches the Arduino reference, which uses millis() there).
//   lino_micros() = a dedicated free-running 1 MHz timer (TIM5, set up by lino_time_init),
//     giving TRUE ~1 us resolution for the encoder RPM dt (the reference uses micros()).
//     Works in Renode (the timer free-runs on virtual time) AND on hardware — unlike the
//     DWT cycle counter, which Renode does not model. MUST be called only after
//     lino_time_init() (TIM5 clock enabled) — i.e. from the running control loop.
static inline uint32_t lino_millis(void) { return (uint32_t)xTaskGetTickCount(); }
static inline uint32_t lino_micros(void) { return (uint32_t)TIM5->CNT; }

void lino_time_init(void);                       // bring up the 1 MHz TIM5 micros base
uint32_t lino_tim_clk_hz(TIM_TypeDef* tim);      // a timer's kernel clock (APB1/APB2 + F4 doubling)

// Enable the bus clock for a TIM / GPIO port (dispatch on the instance).
void lino_tim_clk_enable(TIM_TypeDef* tim);
void lino_gpio_clk_enable(GPIO_TypeDef* port);

#endif // LINO_HAL_H
