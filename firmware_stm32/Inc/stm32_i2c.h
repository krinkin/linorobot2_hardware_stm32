// Thin HAL I2C adapter -- the SOLE place HAL I2C symbols appear in the port. Wraps a
// configured bus (I2cDesc) and exposes register read/write/ping used by the I2C device
// drivers (e.g. the MPU6050 IMU). Mirrors the stm32_encoder/stm32_motor adapter pattern.
#ifndef STM32_I2C_H
#define STM32_I2C_H

#include "stm32f4xx_hal.h"
#include "lino_hal.h"

class I2cBus
{
public:
    explicit I2cBus(const I2cDesc& d) : desc_(d), hi2c_{} {}
    bool init();                                                    // GPIO AF-OD + HAL_I2C_Init; false on HAL error
    bool read_regs(uint8_t dev7, uint8_t reg, uint8_t* buf, uint16_t n);  // burst read from reg
    bool write_reg(uint8_t dev7, uint8_t reg, uint8_t val);              // single-byte reg write
    bool ping(uint8_t dev7);                                            // device-ready probe

private:
    I2cDesc           desc_;
    I2C_HandleTypeDef hi2c_;
};

#endif // STM32_I2C_H
