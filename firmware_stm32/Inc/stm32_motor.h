// Native STM32 motor driver over the reused MotorInterface (spin/invert/sign
// dispatch unchanged). Generic2 = 1 PWM channel + 2 direction GPIOs (the
// lino_base DIFFERENTIAL_DRIVE default). PWM duty = __HAL_TIM_SET_COMPARE; the
// PSC/ARR come from the host-tested pwm_timing(). The ctor ints (pwm/in_a/in_b)
// are LOGICAL ids indexing the board's PWM_DESCRIPTORS / DIR_DESCRIPTORS; a
// negative id marks an unused wheel (no-op). Selected via USE_*_MOTOR_DRIVER.
//
// Plan 5 implements Generic2 only; Generic1/BTS7960/ESC are added when a board
// selects them (their USE_* branch is simply not compiled until then).
#ifndef STM32_MOTOR_H
#define STM32_MOTOR_H

#include "stm32f4xx_hal.h"
#include "config.h"            // USE_*_MOTOR_DRIVER + PWM/DIR descriptor tables
#include "motor_interface.h"

class Generic2 : public MotorInterface
{
public:
    Generic2(float pwm_frequency, int pwm_bits, bool invert,
             int pwm_id, int in_a_id, int in_b_id);
    void init();                 // HAL TIM-PWM + GPIO bring-up; call after HAL_Init

protected:
    void forward(int pwm) override;
    void reverse(int pwm) override;

public:
    void brake() override;

private:
    float             freq_;
    int               bits_;
    int               pwm_id_, in_a_id_, in_b_id_;
    bool              used_;
    uint32_t          arr_;
    uint32_t          channel_;
    GPIO_TypeDef*     dir_a_port_; uint16_t dir_a_pin_;
    GPIO_TypeDef*     dir_b_port_; uint16_t dir_b_pin_;
    TIM_HandleTypeDef htim_;
};

#if defined(USE_GENERIC_2_IN_MOTOR_DRIVER)
  #define Motor Generic2
#elif defined(USE_GENERIC_1_IN_MOTOR_DRIVER)
  #error "Generic1 not yet implemented in the native STM32 port (Plan 5 ships Generic2)."
#elif defined(USE_BTS7960_MOTOR_DRIVER)
  #error "BTS7960 not yet implemented in the native STM32 port (Plan 5 ships Generic2)."
#elif defined(USE_ESC_MOTOR_DRIVER)
  #error "ESC not yet implemented in the native STM32 port (Plan 5 ships Generic2)."
#else
  #error "No motor driver selected: define USE_GENERIC_2_IN_MOTOR_DRIVER."
#endif

#endif // STM32_MOTOR_H
