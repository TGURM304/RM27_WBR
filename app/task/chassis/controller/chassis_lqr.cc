//
// Created by KLnormal on 2026/11/7.
//

#include "chassis_lqr.h"
#include "chassis_data_conifg.h"
#include <algorithm>
#include <cmath>

using namespace Controller;

namespace {
// 将偏航角或角差归入 ±π 范围，跨圈时保留最短转向。
float wrap_angle(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}
}

void ChassisLQR::coef_clc(const float (&basis)[10])
{
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
    lqr_feedback_clc(target_vel, target_omega, state, gain, 0.0f);
}

void ChassisLQR::lqr_feedback_clc(float target_vel, float target_omega, const chassis_state& state,
    const float (*gain)[10], float feed_dis)
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
        target_pos.x - current_pos.x - feed_dis,
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
    output_.T_w_l = tor[0];
    output_.T_w_r = tor[1];
    output_.T_b_l = tor[2];
    output_.T_b_r = tor[3];
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
    if (!enable_flag) {
        output_ = {};
        return;
    }

    // K 和 Ud 共用 poly33 基函数，x 为右腿长，y 为左腿长，单位 m。
    const float x = right_len, y = left_len;
    const float x2 = x * x, y2 = y * y;
    const float basis[10] = {1.0f, x, y, x2, x * y, y2,
                            x2 * x, x2 * y, x * y2, y2 * y};
    coef_clc(basis);
    // 七组实测数据的二次最小二乘拟合，腿长和位移补偿均为 m。
    const float mean_leg_len = 0.5f * (left_len + right_len);
    const float delta_len = std::clamp(mean_leg_len, 0.15f, 0.32f) - 0.23f;
    const float feed_dis = (2.142075f - 6.214766f * delta_len) * delta_len - 0.280264f;
    lqr_feedback_clc(target_vel, target_omega, state, matrix_K, feed_dis);
    forward_feed(basis);
}

void ChassisLQR::forward_feed(const float (&basis)[10])
{
    float ud[4] = {};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 10; ++j)
            ud[i] += kU_dPoly_whole[i][j] * basis[j];

    output_.forward_T_wl = ud[0];
    output_.forward_T_wr = ud[1];
    output_.forward_T_bl = ud[2];
    output_.forward_T_br = ud[3];
    output_.T_w_l += ud[0];
    output_.T_w_r += ud[1];
    output_.T_b_l += ud[2];
    output_.T_b_r += ud[3];
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
