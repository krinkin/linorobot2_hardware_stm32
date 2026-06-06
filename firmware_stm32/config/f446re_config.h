// NUCLEO-F446RE board configuration for the native STM32 port (DIFFERENTIAL_DRIVE).
// Two layers:
//   (1) linorobot SEMANTIC macros (copied from config/lino_base_config.h) consumed
//       by Kinematics / PID / the control loop -- identical names to the Arduino tree.
//   (2) STM32 HARDWARE binding: the timer input clock + per-logical-id descriptor
//       tables (the ONLY F446-specific constants). A second board = a new header
//       with the same macros + its own tables; zero driver/control changes.
//
// The leading int args of the portable Encoder()/Motor() ctors are reinterpreted as
// LOGICAL IDS 0..3 indexing these tables; a negative id marks an unused wheel.
#ifndef F446RE_CONFIG_H
#define F446RE_CONFIG_H

#include "lino_hal.h"   // EncoderDesc / PwmDesc / GpioDesc + HAL symbols

// ---- linorobot semantic macros (Kinematics/PID/control) ----
#define LINO_BASE DIFFERENTIAL_DRIVE
#define USE_GENERIC_2_IN_MOTOR_DRIVER

#define MOTOR_MAX_RPM           140
#define MAX_RPM_RATIO           0.85f
#define MOTOR_OPERATING_VOLTAGE 24
#define MOTOR_POWER_MAX_VOLTAGE 12
#define WHEEL_DIAMETER          0.152f
#define LR_WHEELS_DISTANCE      0.271f

#define COUNTS_PER_REV1 144000
#define COUNTS_PER_REV2 144000
#define COUNTS_PER_REV3 144000
#define COUNTS_PER_REV4 144000

#define K_P 0.6f
#define K_I 0.8f
#define K_D 0.5f

#define PWM_BITS      10
#define PWM_FREQUENCY 20000             // NOTE: unreachable at HSI-16/10-bit (~15.6kHz actual);
                                        // pwm_timing() self-corrects when the PLL clock lands.
#define PWM_MAX ((1 << PWM_BITS) - 1)   // integer form, NOT pow() (Arduino-only/float)
#define PWM_MIN (-PWM_MAX)

#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false
#define MOTOR4_INV false
#define MOTOR1_ENCODER_INV false
#define MOTOR2_ENCODER_INV false
#define MOTOR3_ENCODER_INV false
#define MOTOR4_ENCODER_INV false

// ---- logical-id assignments fed to the portable ctors (negative => unused) ----
// DIFFERENTIAL_DRIVE: only M1/M2 populated; M3/M4 unused.
#define MOTOR1_ENCODER_A 0    // index into ENC_DESCRIPTORS (the _B arg is ignored)
#define MOTOR1_ENCODER_B 0
#define MOTOR2_ENCODER_A 1
#define MOTOR2_ENCODER_B 1
#define MOTOR3_ENCODER_A -1
#define MOTOR3_ENCODER_B -1
#define MOTOR4_ENCODER_A -1
#define MOTOR4_ENCODER_B -1

#define MOTOR1_PWM  0         // index into PWM_DESCRIPTORS
#define MOTOR1_IN_A 0         // index into DIR_DESCRIPTORS
#define MOTOR1_IN_B 1
#define MOTOR2_PWM  1
#define MOTOR2_IN_A 2
#define MOTOR2_IN_B 3
#define MOTOR3_PWM  -1
#define MOTOR3_IN_A -1
#define MOTOR3_IN_B -1
#define MOTOR4_PWM  -1
#define MOTOR4_IN_A -1
#define MOTOR4_IN_B -1

// ---- IMU / MAG selection (mirrors firmware/lib/imu/imu.h + mag.h) ----
// USE_MPU6050_IMU -> stm32_imu.h: #define IMU Mpu6050Imu; AND imu_interface.h getData()
// skips the host gyro-cal subtraction (matching the Arduino chip-calibrated path).
// No USE_*_MAG -> stm32_mag.h falls through to FakeMAG (/imu/mag never published).
#define USE_MPU6050_IMU
#define MPU6050_I2C_ADDR 0x68     // 7-bit; AD0=GND (0x69 if AD0=HIGH)

// ---- STM32 hardware binding (the only F446-specific constants) ----
// (Timer kernel clocks are derived per-timer at runtime via lino_tim_clk_hz(), so there is
//  no board-wide TIMER_CLOCK_HZ to get wrong across the APB1/APB2 domains.)
#define LED_PORT GPIOA
#define LED_PIN  GPIO_PIN_5             // Nucleo LD2 (PA5)

// Encoders (TIM_ENCODERMODE_TI12). TIM2 is 32-bit; TIM3 is 16-bit. TIM5 is avoided
// (its CH1/CH2 collide with TIM2 on PA0/PA1); M4 would use TIM1 (PA8/PA9) if populated.
static const EncoderDesc ENC_DESCRIPTORS[] __attribute__((unused)) = {
    /* 0 M1 */ { TIM2, GPIO_AF1_TIM2, GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1, true  },
    /* 1 M2 */ { TIM3, GPIO_AF2_TIM3, GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7, false },
    /* 2 M3 */ { TIM4, GPIO_AF2_TIM4, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7, false },
    /* 3 M4 */ { TIM1, GPIO_AF1_TIM1, GPIOA, GPIO_PIN_8, GPIOA, GPIO_PIN_9, false },
};

// PWM: one 4-channel advanced timer (TIM8), shared PSC/ARR, AF3 on PC6..PC9.
static const PwmDesc PWM_DESCRIPTORS[] __attribute__((unused)) = {
    /* 0 */ { TIM8, GPIO_AF3_TIM8, GPIOC, GPIO_PIN_6, TIM_CHANNEL_1 },
    /* 1 */ { TIM8, GPIO_AF3_TIM8, GPIOC, GPIO_PIN_7, TIM_CHANNEL_2 },
    /* 2 */ { TIM8, GPIO_AF3_TIM8, GPIOC, GPIO_PIN_8, TIM_CHANNEL_3 },
    /* 3 */ { TIM8, GPIO_AF3_TIM8, GPIOC, GPIO_PIN_9, TIM_CHANNEL_4 },
};

// Direction GPIOs on PC0..PC3 (avoid JTAG PB3/4, BOOT1 PB2, and all TIM/USART pins).
static const GpioDesc DIR_DESCRIPTORS[] __attribute__((unused)) = {
    /* 0 */ { GPIOC, GPIO_PIN_0 }, /* 1 */ { GPIOC, GPIO_PIN_1 },
    /* 2 */ { GPIOC, GPIO_PIN_2 }, /* 3 */ { GPIOC, GPIO_PIN_3 },
};

// IMU I2C bus: I2C1 on PB8(SCL)/PB9(SDA) AF4 (Nucleo D15/D14), 100 kHz. Collision-free vs
// the encoder/PWM/dir/USART2 pins above. (PB6/PB7 are also I2C1-AF4 but used by TIM4.)
static const I2cDesc IMU_I2C_DESC __attribute__((unused)) =
    { I2C1, GPIO_AF4_I2C1, GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9, 100000 };

#endif // F446RE_CONFIG_H
