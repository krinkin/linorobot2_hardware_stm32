// Plain-C boundary to the C++ control module. main.c (C) talks to the control
// loop only through these functions — no C++ type ever crosses the boundary.
//   control_loop_init : construct + HAL-init the encoders/motors/PID/kinematics
//                       (call once, after HAL_Init, from a task so clocks are up)
//   control_loop_tick : run one 50 Hz moveBase cycle (deadman, getRPM, PID, spin,
//                       getVelocities, odom.update)
//   control_set_cmd   : feed a new /cmd_vel (resets the 200 ms deadman timer)
//   control_get_odom  : read the latest pose + body velocities
#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

void control_loop_init(void);
void control_loop_tick(void);
void control_set_cmd(float linear_x, float linear_y, float angular_z);
void control_get_odom(float* x, float* y, float* heading,
                      float* vx, float* vy, float* wz);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_LOOP_H
