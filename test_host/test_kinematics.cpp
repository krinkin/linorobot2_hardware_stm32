#include "doctest.h"
#include "kinematics.h"
#include <cmath>

static Kinematics make_diff() {
    // base, motor_max_rpm, max_rpm_ratio, operating_voltage,
    // power_max_voltage, wheel_diameter, wheels_y_distance
    return Kinematics(Kinematics::DIFFERENTIAL_DRIVE, 100, 1.0f,
                      12.0f, 12.0f, 0.1f, 0.3f);
}

TEST_CASE("Kinematics: max RPM derives from voltage ratio and ratio factor") {
    // 9V / 12V * 100 rpm * 1.0 ratio = 75 rpm  (non-trivial ratio)
    Kinematics kin(Kinematics::DIFFERENTIAL_DRIVE, 100, 1.0f,
                   12.0f, 9.0f, 0.1f, 0.3f);
    CHECK(kin.getMaxRPM() == doctest::Approx(75.0));
}

TEST_CASE("Kinematics: driving straight gives equal left/right wheel RPM") {
    Kinematics kin = make_diff();
    Kinematics::rpm r = kin.getRPM(0.1f, 0.0f, 0.0f); // 0.1 m/s forward
    // 0.1 m/s -> 6 m/min / 0.3141593 m = 19.0986 rpm
    CHECK(r.motor1 == doctest::Approx(19.0986).epsilon(0.001));
    CHECK(r.motor2 == doctest::Approx(r.motor1));
}

TEST_CASE("Kinematics: excessive command saturates to max RPM") {
    Kinematics kin = make_diff();
    Kinematics::rpm r = kin.getRPM(10.0f, 0.0f, 0.0f); // way over max
    CHECK(r.motor1 == doctest::Approx(100.0));
    CHECK(r.motor2 == doctest::Approx(100.0));
}

TEST_CASE("Kinematics: pure rotation is antisymmetric across the axle") {
    Kinematics kin = make_diff();
    Kinematics::rpm r = kin.getRPM(0.0f, 0.0f, 1.0f); // 1 rad/s
    // tangential = 1*(0.3/2)=0.15 m/s -> 9 m/min / 0.3141593 = 28.6479 rpm
    CHECK(r.motor2 == doctest::Approx(28.6479).epsilon(0.001));
    CHECK(r.motor1 == doctest::Approx(-r.motor2));
}

TEST_CASE("Kinematics: getVelocities round-trips a straight-line command") {
    Kinematics kin = make_diff();
    Kinematics::velocities v = kin.getVelocities(19.0986f, 19.0986f, 0.0f, 0.0f);
    CHECK(v.linear_x == doctest::Approx(0.1).epsilon(0.001));
    CHECK(std::fabs(v.angular_z) < 1e-4f);
}
