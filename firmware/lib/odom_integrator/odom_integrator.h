// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ODOM_INTEGRATOR_H
#define ODOM_INTEGRATOR_H

#include <cmath>

// Pure dead-reckoning integrator + Euler->quaternion conversion.
// No Arduino, no ROS, no micro-ROS dependencies -> host-unit-testable.
class OdomIntegrator
{
    public:
        float x = 0.0f;
        float y = 0.0f;
        float heading = 0.0f;

        void update(float vel_dt, float linear_vel_x, float linear_vel_y,
                    float angular_vel_z)
        {
            float delta_heading = angular_vel_z * vel_dt;        // radians
            // Forward Euler: integrate position using heading at start of interval
            float cos_h = std::cos(heading);
            float sin_h = std::sin(heading);
            float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt; // m
            float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt; // m

            x += delta_x;
            y += delta_y;
            heading += delta_heading;
        }

        // q layout: q[0]=w, q[1]=x, q[2]=y, q[3]=z
        static void euler_to_quat(float roll, float pitch, float yaw, float* q)
        {
            float cy = std::cos(yaw * 0.5f);
            float sy = std::sin(yaw * 0.5f);
            float cp = std::cos(pitch * 0.5f);
            float sp = std::sin(pitch * 0.5f);
            float cr = std::cos(roll * 0.5f);
            float sr = std::sin(roll * 0.5f);

            q[0] = cy * cp * cr + sy * sp * sr;
            q[1] = cy * cp * sr - sy * sp * cr;
            q[2] = sy * cp * sr + cy * sp * cr;
            q[3] = sy * cp * cr - cy * sp * sr;
        }
};

#endif // ODOM_INTEGRATOR_H
