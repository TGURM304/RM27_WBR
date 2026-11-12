//
// Created by KLnormal on 2026/11/7.
//

#include "chassis_lqr.h"
#include "chassis_data_conifg.h"
#include <cmath>

using namespace Controller;

namespace {
// 将偏航角或角差归入 ±π 范围，跨圈时保留最短转向。
float wrap_angle(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}
}

void ChassisLQR::coef_clc(float left_len, float right_len)
{
    // 当前导出系数约定：x 为右腿长，y 为左腿长，单位 m。
    const float x = right_len, y = left_len;
    const float x2 = x * x, y2 = y * y;
    const float basis[10] = {1.0f, x, y, x2, x * y, y2,
                            x2 * x, x2 * y, x * y2, y2 * y};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 10; ++j) {
            matrix_K[i][j] = 0.0f;
            for (int n = 0; n < 10; ++n)
                matrix_K[i][j] += kMatureLqrPoly_whole[i][j][n] * basis[n];
        }
    }
}

void ChassisLQR::lqr_clc_staticK(float target_vel, float target_omega, const chassis_state& state,
    const float (*gain)[10])
{
    current_pos = state;
    current_pos.x -= zero_pos.x;
    current_pos.phi = wrap_angle(state.phi - zero_pos.phi);
    output_ = {};
    if (!enable_flag) return;
    if (!reference_ready) {
        target_pos.x = current_pos.x;
        target_pos.phi = current_pos.phi;
        reference_ready = true;
    }
    constexpr float dt = 1.0f / WHEEL_CHASSIS_HZ;
    target_pos.x += target_vel * dt;
    target_pos.phi = wrap_angle(target_pos.phi + target_omega * dt);
    target_pos.dot_x = target_vel;
    target_pos.dot_phi = target_omega;

    const float error[10] = {
        target_pos.x - current_pos.x - 0.3f,
        target_pos.dot_x - current_pos.dot_x,
        wrap_angle(target_pos.phi - current_pos.phi),
        target_pos.dot_phi - current_pos.dot_phi,
        -current_pos.left_theta,
        -current_pos.dot_left_theta,
        -current_pos.right_theta,
        -current_pos.dot_right_theta,
        -current_pos.theta,
        -current_pos.dot_theta
    };
    float tor[4] = {};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 10; ++j)
            tor[i] += gain[i][j] * error[j];
    output_ = {tor[0], tor[1], tor[2], tor[3]};
}

void ChassisLQR::lqr_clc_bench(float target_vel, float target_omega, chassis_state state,
    const float (*gain)[10])
{
    state.left_theta = 0.0f;
    state.dot_left_theta = 0.0f;
    state.right_theta = 0.0f;
    state.dot_right_theta = 0.0f;
    lqr_clc_staticK(target_vel, target_omega, state, gain);
}

void ChassisLQR::lqr_clc_fitted(float target_vel, float target_omega, const chassis_state& state,
    float left_len, float right_len)
{
    if (enable_flag) coef_clc(left_len, right_len);
    lqr_clc_staticK(target_vel, target_omega, state, matrix_K);
}

void ChassisLQR::lqr_set_zero(chassis_state zero_state)
{
    zero_pos = {};
    zero_pos.x = zero_state.x;
    zero_pos.phi = zero_state.phi;
    target_pos = {};
    output_ = {};
    reference_ready = false;
}

void ChassisLQR::lqr_disable()
{
    enable_flag = false;
    target_pos = {};
    output_ = {};
    reference_ready = false;
}

void ChassisLQR::lqr_enable(const chassis_state& state)
{
    if (enable_flag) return;
    lqr_set_zero(state);
    enable_flag = true;
}
