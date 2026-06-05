#include "doctest.h"
#include "imu_math.h"

// imu_math: pure raw-register -> SI conversion (firmware_stm32/Inc/imu_math.h).

TEST_CASE("be16: big-endian signed 16-bit from a register byte pair") {
    CHECK(be16(0x40, 0x00) == 16384);
    CHECK(be16(0x00, 0x00) == 0);
    CHECK(be16(0xFF, 0xFF) == -1);
    CHECK(be16(0xFF, 0x80) == -128);
    CHECK(be16(0x7F, 0xFF) == 32767);    // max positive
    CHECK(be16(0x80, 0x00) == -32768);   // min negative
}

TEST_CASE("mpu6050 accel: +/-2g range, 16384 LSB/g -> m/s^2") {
    CHECK(mpu6050_accel_lsb_to_ms2(16384)  == doctest::Approx(9.81f));    // +1 g
    CHECK(mpu6050_accel_lsb_to_ms2(-16384) == doctest::Approx(-9.81f));
    CHECK(mpu6050_accel_lsb_to_ms2(0)      == doctest::Approx(0.0f));
}

TEST_CASE("mpu6050 gyro: +/-250 dps range, 131 LSB/dps -> rad/s") {
    CHECK(mpu6050_gyro_lsb_to_rads(131)  == doctest::Approx(kDegToRad));   // 1 deg/s
    CHECK(mpu6050_gyro_lsb_to_rads(-131) == doctest::Approx(-kDegToRad));
    CHECK(mpu6050_gyro_lsb_to_rads(0)    == doctest::Approx(0.0f));
}

TEST_CASE("range-parameterized variants match the MPU6050 fixed forms") {
    CHECK(accel_lsb_to_ms2(16384, 16384.0f) == doctest::Approx(mpu6050_accel_lsb_to_ms2(16384)));
    CHECK(gyro_lsb_to_rads(131, 131.0f)     == doctest::Approx(mpu6050_gyro_lsb_to_rads(131)));
}
