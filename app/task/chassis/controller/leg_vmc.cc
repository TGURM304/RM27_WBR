//
// Created by KLnormal on 2026/11/6.
//

#include "leg_vmc.h"
#include "chassis_data_conifg.h"
#include <cmath>
#include <numbers>

namespace {
struct vmc_coefficients {
    float len, j_phi, j_len;
};

vmc_coefficients vmc_coeff_clc(float theta1, float theta2) {
    constexpr float a = LEG_LEN_AG, b = LEG_LEN_GW;
    constexpr float k = LEG_LEN_AD / LEG_LEN_CD;
    constexpr float two_pi = 2.0f * std::numbers::pi_v<float>;
    // theta2 大腿顺时针转到 theta1 小腿，范围 [0, 2pi)。
    float delta = fmodf(theta2 - theta1, two_pi);
    if (delta < 0.0f) delta += two_pi;
    const float half_angle = 0.5f * delta;
    const float sin_half = sinf(half_angle), cos_half = cosf(half_angle);
    const float sin_beta = k * sin_half;
    const float cos_beta = sqrtf(1.0f - sin_beta * sin_beta);
    const float sin_q = sin_half * cos_beta + cos_half * sin_beta;
    const float cos_q = cos_half * cos_beta - sin_half * sin_beta;
    const float h = 1.0f + k * cos_half / cos_beta;
    const float len_squared = a * a + b * b + 2.0f * a * b * cos_q;
    const float len = sqrtf(len_squared);
    const float t = b * (b + a * cos_q) / len_squared;
    return {len, t * h * 0.5f, a * b * h * sin_q / (2.0f * len)};
}

std::pair<float,float> torque_clc(const vmc_coefficients& c, float tor_M, float force_L) {
    return {c.j_phi * tor_M + c.j_len * force_L,
            (1.0f - c.j_phi) * tor_M - c.j_len * force_L};
}
}

//first:小腿电机扭矩，second:大腿电机扭矩
std::pair<float,float> vmc_pol_clc(float theta1, float theta2,
    float tor_M, float force_L) {
    return torque_clc(vmc_coeff_clc(theta1, theta2), tor_M, force_L);
}

std::pair<float,float> vmc_co_clc(float theta1, float theta2, float force_x,
    float force_y, float leg_phi) {
    const auto c = vmc_coeff_clc(theta1, theta2);
    const float cos_phi = cosf(leg_phi), sin_phi = sinf(leg_phi);
    const float force_L = force_x * cos_phi + force_y * sin_phi;
    const float tor_M = c.len * (-force_x * sin_phi + force_y * cos_phi);
    return torque_clc(c, tor_M, force_L);
}
