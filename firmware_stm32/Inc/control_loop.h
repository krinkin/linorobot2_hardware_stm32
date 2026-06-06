// Plain-C boundary to the C++ control module. main.c (C) talks to the control
// loop only through these functions -- no C++ type ever crosses the boundary.
//   control_loop_init : construct + HAL-init the encoders/motors/PID/kinematics
//                       (call once, after HAL_Init, from a task so clocks are up)
//   control_loop_tick : run one 50 Hz moveBase cycle (deadman, getRPM, PID, spin,
//                       getVelocities, odom.update)
//   control_set_cmd   : feed a new /cmd_vel (resets the 200 ms deadman timer)
//   control_get_odom  : read the latest pose + body velocities
//   control_get_imu   : read the latest IMU sample (numeric fields only)
#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <sensor_msgs/msg/imu.h>

#ifdef __cplusplus
extern "C" {
#endif

void control_loop_init(void);
void control_loop_tick(void);
void control_set_cmd(float linear_x, float linear_y, float angular_z);
void control_get_odom(float* x, float* y, float* heading,
                      float* vx, float* vy, float* wz);

// Fills ONLY the numeric fields (orientation, angular_velocity, linear_acceleration,
// and the three covariance arrays) from the latest control-task IMU read, under a
// critical section. MUST NOT touch out->header -- the caller (uros_task) owns the
// header.frame_id String.
void control_get_imu(sensor_msgs__msg__Imu* out);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_LOOP_H
