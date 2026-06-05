// Pure, HAL/ROS/Arduino-free IMU sample math for the native STM32 port. Converts raw
// 16-bit sensor registers to SI units; host-unit-testable (Tier A), mirroring
// encoder_math.h / pwm_timing.h. The driver class assembles the ROS Vector3 from these.
//
// MPU6050 defaults (the ranges the driver pins in startSensor): accel +/-2g => 16384 LSB/g,
// gyro +/-250 dps => 131 LSB/dps. Registers are big-endian (MSB first).
#ifndef IMU_MATH_H
#define IMU_MATH_H

#include <stdint.h>

constexpr float kGToMs2   = 9.81f;                                  // == IMUInterface::g_to_accel_
constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;       // DEG_TO_RAD is Arduino-core, absent here

// Big-endian (MSB-first) signed 16-bit from a register byte pair.
inline int16_t be16(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

// MPU6050 default-range conversions.
inline float mpu6050_accel_lsb_to_ms2(int16_t raw) { return raw * (1.0f / 16384.0f) * kGToMs2; }
inline float mpu6050_gyro_lsb_to_rads(int16_t raw) { return raw * (1.0f / 131.0f)   * kDegToRad; }

// Range-parameterized variants for future ranges / MPU9250.
inline float accel_lsb_to_ms2(int16_t raw, float lsb_per_g)   { return raw / lsb_per_g   * kGToMs2; }
inline float gyro_lsb_to_rads(int16_t raw, float lsb_per_dps) { return raw / lsb_per_dps * kDegToRad; }

#endif // IMU_MATH_H
