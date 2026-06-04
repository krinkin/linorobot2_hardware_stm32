#include "doctest.h"
#include "odom_integrator.h"

TEST_CASE("OdomIntegrator: straight motion advances x only") {
    OdomIntegrator odo;
    odo.update(/*dt*/1.0f, /*vx*/1.0f, /*vy*/0.0f, /*wz*/0.0f);
    CHECK(odo.x == doctest::Approx(1.0));
    CHECK(odo.y == doctest::Approx(0.0));
    CHECK(odo.heading == doctest::Approx(0.0));
}

TEST_CASE("OdomIntegrator: rotate 90deg then drive forward moves along +y") {
    OdomIntegrator odo;
    odo.update(1.0f, 0.0f, 0.0f, 1.5707963f); // +pi/2 rad/s for 1 s
    CHECK(odo.heading == doctest::Approx(1.5707963));
    odo.update(1.0f, 1.0f, 0.0f, 0.0f);       // 1 m/s forward, now facing +y
    CHECK(odo.x == doctest::Approx(0.0).epsilon(0.001));
    CHECK(odo.y == doctest::Approx(1.0).epsilon(0.001));
}

TEST_CASE("OdomIntegrator: euler_to_quat identity") {
    float q[4];
    OdomIntegrator::euler_to_quat(0.0f, 0.0f, 0.0f, q);
    CHECK(q[0] == doctest::Approx(1.0)); // w
    CHECK(q[1] == doctest::Approx(0.0));
    CHECK(q[2] == doctest::Approx(0.0));
    CHECK(q[3] == doctest::Approx(0.0));
}

TEST_CASE("OdomIntegrator: euler_to_quat 180deg yaw") {
    float q[4];
    OdomIntegrator::euler_to_quat(0.0f, 0.0f, 3.1415927f, q);
    CHECK(q[0] == doctest::Approx(0.0).epsilon(0.001)); // w
    CHECK(q[1] == doctest::Approx(0.0).epsilon(0.001)); // x
    CHECK(q[2] == doctest::Approx(0.0).epsilon(0.001)); // y
    CHECK(q[3] == doctest::Approx(1.0).epsilon(0.001)); // z
}
