#include "stm32_i2c.h"

// Per-transfer timeout. Kept well under the 20 ms control tick so a transient stall on a
// faulted bus can't blow the loop budget. NOTE: the HAL's leading BUSY-flag wait uses a
// fixed 25 ms internally, so a truly wedged bus still needs the gating in control_loop +
// (on real hardware) an SCL bus-recovery sequence — a documented hardware-robustness item.
#define I2C_TIMEOUT_MS 8u

// Clock + GPIO are owned here (adapter-owns-GPIO convention), so the HAL MspInit hook
// is a strong no-op — otherwise HAL_I2C_Init would call the weak default and a second
// config site could silently appear.
extern "C" void HAL_I2C_MspInit(I2C_HandleTypeDef*) {}

bool I2cBus::init()
{
    lino_i2c_clk_enable(desc_.i2c);
    lino_gpio_clk_enable(desc_.port_scl);
    lino_gpio_clk_enable(desc_.port_sda);

    // I2C pins MUST be open-drain. Internal pull-ups let the bus idle high in Renode with
    // no external resistors; real boards still need external 4.7k at 100 kHz+.
    GPIO_InitTypeDef g = {};
    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = desc_.af;
    g.Pin = desc_.pin_scl; HAL_GPIO_Init(desc_.port_scl, &g);
    g.Pin = desc_.pin_sda; HAL_GPIO_Init(desc_.port_sda, &g);

    hi2c_.Instance             = desc_.i2c;
    hi2c_.Init.ClockSpeed      = desc_.clock_speed;   // F4 classic field; HAL_I2C_Init derives CCR from PCLK1
    hi2c_.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c_.Init.OwnAddress1     = 0;
    hi2c_.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c_.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c_.Init.OwnAddress2     = 0;
    hi2c_.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c_.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    return HAL_I2C_Init(&hi2c_) == HAL_OK;   // false if PCLK1 too low for the requested speed, etc.
}

// HAL takes the 8-bit address; a 7-bit device addr is shifted left by 1 (0x68 -> 0xD0).
// Repeated-START register read (the canonical MPU6050 burst read).
bool I2cBus::read_regs(uint8_t dev7, uint8_t reg, uint8_t* buf, uint16_t n)
{
    return HAL_I2C_Mem_Read(&hi2c_, (uint16_t)(dev7 << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, buf, n, I2C_TIMEOUT_MS) == HAL_OK;
}

bool I2cBus::write_reg(uint8_t dev7, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c_, (uint16_t)(dev7 << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

bool I2cBus::ping(uint8_t dev7)
{
    return HAL_I2C_IsDeviceReady(&hi2c_, (uint16_t)(dev7 << 1), 2, I2C_TIMEOUT_MS) == HAL_OK;
}
