// Native STM32 IMU drivers (replaces the Arduino-bound default_imu.h, which pulls
// I2Cdevlib + Wire). Provides a register-level MPU6050 driver over the HAL I2C adapter
// and the verbatim FakeIMU fallback, both satisfying the reused IMUInterface. The
// USE_*_IMU -> #define IMU <Class> selector mirrors firmware/lib/imu/imu.h.
//
// Include order matters: imu_interface.h calls delay() (Arduino.h) and
// micro_ros_string_utilities_set() (micro_ros_utilities) without including them itself.
#ifndef STM32_IMU_H
#define STM32_IMU_H

#include "Arduino.h"                              // delay()
#include <micro_ros_utilities/string_utilities.h> // micro_ros_string_utilities_set()
#include "imu_interface.h"                        // reused verbatim (getData/init/calibrateGyro)
#include "imu_math.h"
#include "stm32_i2c.h"

// MPU6050 registers (default ranges: accel +/-2g, gyro +/-250 dps).
#define MPU6050_REG_WHO_AM_I    0x75
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_WHO_AM_I_VALUE  0x68

// 6-DoF MPU6050 over real HAL I2C. getData()/init()/calibrateGyro() are inherited from
// IMUInterface; this class implements only the three pure virtuals.
class Mpu6050Imu : public IMUInterface
{
public:
    Mpu6050Imu(I2cBus* bus, uint8_t addr7) : bus_(bus), addr_(addr7) {}

    bool startSensor() override
    {
        uint8_t who = 0;
        if (!bus_->read_regs(addr_, MPU6050_REG_WHO_AM_I, &who, 1)) return false;
        if (who != MPU6050_WHO_AM_I_VALUE) return false;
        // PWR_MGMT_1 = 0x01: wake (SLEEP=0) + CLKSEL=1 (PLL ref X-gyro), matching the Arduino
        // setClockSource(MPU6050_CLOCK_PLL_XGYRO) path (a more stable clock than internal RC).
        if (!bus_->write_reg(addr_, MPU6050_REG_PWR_MGMT_1, 0x01)) return false;
        if (!bus_->write_reg(addr_, MPU6050_REG_GYRO_CONFIG, 0x00))  return false; // +/-250 dps -> 131 LSB/dps
        if (!bus_->write_reg(addr_, MPU6050_REG_ACCEL_CONFIG, 0x00)) return false; // +/-2g     -> 16384 LSB/g
        delay(50);   // gyro analog start-up settle before the first samples
        return true;
    }

    // Native gyro-bias calibration (the Arduino path calibrates the chip's offset registers;
    // this port has no I2Cdevlib, so we average N stationary samples and subtract in software).
    // Hides the non-virtual IMUInterface::init() — imu is typed Mpu6050Imu* (via the IMU macro),
    // and getData() skips the base gyro_cal_ subtraction under USE_MPU6050_IMU, so this is the
    // sole bias correction. MUST run with the robot stationary (it does: before any /cmd_vel,
    // deadman holding the base).
    bool init()
    {
        if (!startSensor()) return false;
        geometry_msgs__msg__Vector3 sum{};
        const int N = 40;
        for (int i = 0; i < N; ++i) {
            geometry_msgs__msg__Vector3 g = readGyroscope();   // calibrated_==false -> raw
            sum.x += g.x; sum.y += g.y; sum.z += g.z;
            delay(50);
        }
        bias_.x = sum.x / N; bias_.y = sum.y / N; bias_.z = sum.z / N;
        calibrated_ = true;
        return true;
    }

    geometry_msgs__msg__Vector3 readAccelerometer() override
    {
        uint8_t b[6];
        last_read_ok_ = bus_->read_regs(addr_, MPU6050_REG_ACCEL_XOUT_H, b, 6);
        if (last_read_ok_) {
            accel_.x = mpu6050_accel_lsb_to_ms2(be16(b[0], b[1]));
            accel_.y = mpu6050_accel_lsb_to_ms2(be16(b[2], b[3]));
            accel_.z = mpu6050_accel_lsb_to_ms2(be16(b[4], b[5]));
        }
        return accel_;   // last-good on a transient I2C failure (see readOk())
    }

    geometry_msgs__msg__Vector3 readGyroscope() override
    {
        uint8_t b[6];
        last_read_ok_ = bus_->read_regs(addr_, MPU6050_REG_GYRO_XOUT_H, b, 6);
        if (last_read_ok_) {
            gyro_.x = mpu6050_gyro_lsb_to_rads(be16(b[0], b[1]));
            gyro_.y = mpu6050_gyro_lsb_to_rads(be16(b[2], b[3]));
            gyro_.z = mpu6050_gyro_lsb_to_rads(be16(b[4], b[5]));
        }
        if (calibrated_) { gyro_.x -= bias_.x; gyro_.y -= bias_.y; gyro_.z -= bias_.z; }
        return gyro_;
    }

    bool readOk() const { return last_read_ok_; }   // last register read succeeded (health signal)

private:
    I2cBus* bus_;
    uint8_t addr_;
    geometry_msgs__msg__Vector3 accel_{};
    geometry_msgs__msg__Vector3 gyro_{};
    geometry_msgs__msg__Vector3 bias_{};
    bool calibrated_ = false;
    bool last_read_ok_ = false;
};

// Verbatim from firmware/lib/imu/default_imu.h — the no-IMU fallback.
class FakeIMU : public IMUInterface
{
public:
    FakeIMU() {}
    bool startSensor() override { return true; }
    geometry_msgs__msg__Vector3 readAccelerometer() override { return accel_; }
    geometry_msgs__msg__Vector3 readGyroscope() override { return gyro_; }
    bool readOk() const { return true; }   // no bus -> always "ok" (uniform with Mpu6050Imu)
private:
    geometry_msgs__msg__Vector3 accel_{};
    geometry_msgs__msg__Vector3 gyro_{};
};

// Selector (mirror of firmware/lib/imu/imu.h). Add an arm + concrete class per new chip.
#ifdef USE_MPU6050_IMU
#define IMU Mpu6050Imu
#endif

#ifndef IMU
#define USE_FAKE_IMU
#define IMU FakeIMU
#endif

#endif // STM32_IMU_H
