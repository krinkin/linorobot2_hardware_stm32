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

#include "odometry.h"

Odometry::Odometry()
{
    odom_msg_.header.frame_id = micro_ros_string_utilities_set(odom_msg_.header.frame_id, "odom");
    odom_msg_.child_frame_id = micro_ros_string_utilities_set(odom_msg_.child_frame_id, "base_footprint");
}

void Odometry::update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z)
{
    integrator_.update(vel_dt, linear_vel_x, linear_vel_y, angular_vel_z);

    const float pose_cov[6] = POSE_COV;
    const float twist_cov[6] = TWIST_COV;

    //calculate robot's heading in quaternion angle
    float q[4];
    OdomIntegrator::euler_to_quat(0, 0, integrator_.heading, q);

    //robot's position in x,y, and z
    odom_msg_.pose.pose.position.x = integrator_.x;
    odom_msg_.pose.pose.position.y = integrator_.y;
    odom_msg_.pose.pose.position.z = 0.0;

    //robot's heading in quaternion
    odom_msg_.pose.pose.orientation.x = (double) q[1];
    odom_msg_.pose.pose.orientation.y = (double) q[2];
    odom_msg_.pose.pose.orientation.z = (double) q[3];
    odom_msg_.pose.pose.orientation.w = (double) q[0];

    odom_msg_.pose.covariance[0] = pose_cov[0];
    odom_msg_.pose.covariance[7] = pose_cov[1];
    odom_msg_.pose.covariance[14] = pose_cov[2];
    odom_msg_.pose.covariance[21] = pose_cov[3];
    odom_msg_.pose.covariance[28] = pose_cov[4];
    odom_msg_.pose.covariance[35] = pose_cov[5];

    //linear speed from encoders
    odom_msg_.twist.twist.linear.x = linear_vel_x;
    odom_msg_.twist.twist.linear.y = linear_vel_y;
    odom_msg_.twist.twist.linear.z = 0.0;

    //angular speed from encoders
    odom_msg_.twist.twist.angular.x = 0.0;
    odom_msg_.twist.twist.angular.y = 0.0;
    odom_msg_.twist.twist.angular.z = angular_vel_z;

    odom_msg_.twist.covariance[0] = twist_cov[0];
    odom_msg_.twist.covariance[7] = twist_cov[1];
    odom_msg_.twist.covariance[14] = twist_cov[2];
    odom_msg_.twist.covariance[21] = twist_cov[3];
    odom_msg_.twist.covariance[28] = twist_cov[4];
    odom_msg_.twist.covariance[35] = twist_cov[5];
}

nav_msgs__msg__Odometry Odometry::getData()
{
    return odom_msg_;
}

