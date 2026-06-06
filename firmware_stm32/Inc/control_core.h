// Pure (HAL-free) control step -- the moveBase() math, extracted so it is
// host-unit-testable with a FakeMotor and injected wheel RPMs. The HAL encoder
// reads, the 50 Hz cadence and the deadman timer live in control_loop.cpp; this
// header holds only: cmd -> Kinematics::getRPM -> PID -> Motor::spin ->
// Kinematics::getVelocities -> OdomIntegrator::update.
//
// Reuses the portable, already-host-tested Kinematics / PID / OdomIntegrator and
// the MotorInterface dispatch verbatim, so behavior matches the Arduino firmware.
#ifndef CONTROL_CORE_H
#define CONTROL_CORE_H

#include "kinematics.h"
#include "pid.h"
#include "odom_integrator.h"
#include "motor_interface.h"

class ControlCore
{
public:
    // Any motor/pid pointer may be null (an unused wheel) -> that wheel is skipped.
    ControlCore(Kinematics& kin, OdomIntegrator& odom,
                PID* p0, PID* p1, PID* p2, PID* p3,
                MotorInterface* m0, MotorInterface* m1,
                MotorInterface* m2, MotorInterface* m3)
        : kin_(kin), odom_(odom)
    {
        pid_[0] = p0; pid_[1] = p1; pid_[2] = p2; pid_[3] = p3;
        mot_[0] = m0; mot_[1] = m1; mot_[2] = m2; mot_[3] = m3;
    }

    // One control cycle. measured_rpm[4] = wheel RPMs from the encoders;
    // dt = seconds since the previous call (for odometry integration).
    Kinematics::velocities step(float cmd_x, float cmd_y, float cmd_wz,
                                const float measured_rpm[4], float dt)
    {
        Kinematics::rpm req = kin_.getRPM(cmd_x, cmd_y, cmd_wz);
        const float target[4] = { req.motor1, req.motor2, req.motor3, req.motor4 };

        for (int i = 0; i < 4; ++i)
            if (mot_[i] && pid_[i])
                mot_[i]->spin((int)pid_[i]->compute(target[i], measured_rpm[i]));

        Kinematics::velocities v = kin_.getVelocities(
            measured_rpm[0], measured_rpm[1], measured_rpm[2], measured_rpm[3]);
        odom_.update(dt, v.linear_x, v.linear_y, v.angular_z);
        return v;
    }

private:
    Kinematics&     kin_;
    OdomIntegrator& odom_;
    PID*            pid_[4];
    MotorInterface* mot_[4];
};

#endif // CONTROL_CORE_H
