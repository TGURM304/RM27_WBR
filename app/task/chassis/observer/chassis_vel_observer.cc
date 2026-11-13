//
// Created by KLnormal on 2026/11/13.
//

#include "chassis_vel_observer.h"

#include "chassis_data_conifg.h"
#include "math/matrix.h"
#include <cmath>

#include "chassis_coor.h"

namespace
{
float wheel_mount_rate(const Coordinator::leg_state& leg, const Observer::joints& big_joint)
{
    const float sin_phi = std::sin(leg.current_rad);
    const float cos_phi = std::cos(leg.current_rad);
    const float ag_x = LEG_LEN_AG * std::cos(big_joint.pos_);
    const float ag_y = LEG_LEN_AG * std::sin(big_joint.pos_);
    const float gw_x = leg.current_len * cos_phi - ag_x;
    const float gw_y = leg.current_len * sin_phi - ag_y;

    // GW = AW − AG，角速度为 (GW × dGW/dt) / |GW|²。
    const float tangent_speed = leg.current_len * leg.current_omega;
    const float gw_vx = leg.current_speed * cos_phi - tangent_speed * sin_phi + ag_y * big_joint.vel_;
    const float gw_vy = leg.current_speed * sin_phi + tangent_speed * cos_phi - ag_x * big_joint.vel_;
    return (gw_x * gw_vy - gw_y * gw_vx) / (LEG_LEN_GW * LEG_LEN_GW);
}
}

namespace Observer
{
void ChassisVelObserver::enable()
{
    enabled_ = true;
}

void ChassisVelObserver::disable()
{
    enabled_ = false;
    clear();
}

void ChassisVelObserver::clear()
{
    acc_x = 0.0f;
    vel_wheel = 0.0f;
    vel_wheel_left = 0.0f;
    vel_wheel_right = 0.0f;
}

void ChassisVelObserver::chassis_move_clc(Coordinator::leg_state left_leg, Coordinator::leg_state right_leg, chassis_basedata chassis)
{
    if (!enabled_) return;
    const auto& posture = chassis.body;

    // 水平坐标系随航向转动，X 轴沿机头的水平投影。
    const auto body_to_level = math::matrix<3, 3>::rot_euler(
        posture.body_roll, posture.body_theta, 0.0f);
    const math::matrix<3, 1> body_acc{posture.body_x_acc, posture.body_y_acc, posture.body_z_acc};
    acc_x = (body_to_level * body_acc)[0];

    const float left_mount_rate = wheel_mount_rate(left_leg, chassis.j3);
    const float right_mount_rate = wheel_mount_rate(right_leg, chassis.j2);
    const float body_axle_rate = posture.body_gyro_pitch;
    vel_wheel_left = WHEEL_RADIUS * (chassis.motor_left.angular_velocity + left_mount_rate + body_axle_rate);
    vel_wheel_right = WHEEL_RADIUS * (chassis.motor_right.angular_velocity + right_mount_rate + body_axle_rate);
    vel_wheel = 0.5f * (vel_wheel_left + vel_wheel_right);
}
}
