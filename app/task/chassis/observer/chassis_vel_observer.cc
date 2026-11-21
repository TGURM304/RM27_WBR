//
// Created by KLnormal on 2026/11/13.
//

#include "chassis_vel_observer.h"

#include "chassis_data_conifg.h"
#include "math/matrix.h"
#include <cmath>
#include <limits>

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

float wheel_relative_velocity(const Coordinator::leg_state& leg, const Observer::body_posture& posture,
                              const math::matrix<3, 3>& body_to_level)
{
    const float cos_phi = std::cos(leg.current_rad), sin_phi = std::sin(leg.current_rad);
    const float x = leg.current_len * cos_phi, z = -leg.current_len * sin_phi;
    const float tangent_speed = leg.current_len * leg.current_omega;

    // 机体 X 向前、Z 向上。左右安装点的横向偏置在轮轴中心平均时相消。
    const math::matrix<3, 1> relative_velocity{
        leg.current_speed * cos_phi - tangent_speed * sin_phi + posture.body_gyro_pitch * z,
        posture.body_gyro_yaw * x - posture.body_gyro_roll * z,
        -leg.current_speed * sin_phi - tangent_speed * cos_phi - posture.body_gyro_pitch * x
    };
    return (body_to_level * relative_velocity)[0];
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
    kalman_vel = 0.0f;
    kalman_acc = 0.0f;
    wheel_relative_vel_ = 0.0f;
    p_vv_ = p_va_ = p_aa_ = 0.0;
    kalman_initialized_ = false;
}

float ChassisVelObserver::update(const Coordinator::leg_state& left_leg, const Coordinator::leg_state& right_leg,
                                 const chassis_basedata& chassis, float sigma_Q, float sigma_R_vel, float sigma_R_acc)
{
    if (!enabled_) return 0.0f;
    const auto& posture = chassis.body;

    // 水平坐标系随航向转动，X 轴沿机头的水平投影。
    const auto body_to_level = math::matrix<3, 3>::rot_euler(
        posture.body_roll, posture.body_theta, 0.0f);
    const math::matrix<3, 1> body_acc{posture.body_x_acc, posture.body_y_acc, posture.body_z_acc};
    const float acc_x = (body_to_level * body_acc)[0];

    const float left_mount_rate = wheel_mount_rate(left_leg, chassis.j3);
    const float right_mount_rate = wheel_mount_rate(right_leg, chassis.j2);
    const float body_axle_rate = posture.body_gyro_pitch;
    const float vel_wheel_left = WHEEL_RADIUS * (chassis.motor_left.angular_velocity + left_mount_rate + body_axle_rate);
    const float vel_wheel_right = WHEEL_RADIUS * (chassis.motor_right.angular_velocity + right_mount_rate + body_axle_rate);
    const float vel_wheel = 0.5f * (vel_wheel_left + vel_wheel_right);
    const float relative_vel = 0.5f * (wheel_relative_velocity(left_leg, posture, body_to_level)
                                    + wheel_relative_velocity(right_leg, posture, body_to_level));
    const float body_vel = vel_wheel - relative_vel;
    if (chassis_kalman(body_vel, acc_x, sigma_Q, sigma_R_vel, sigma_R_acc) && std::isfinite(relative_vel)) {
        wheel_relative_vel_ = relative_vel;
    }
    return body_vel;
}

bool ChassisVelObserver::chassis_kalman(float body_vel, float acc_x,
                                        float sigma_Q, float sigma_R_vel, float sigma_R_acc)
{
    constexpr float inf = std::numeric_limits<float>::infinity();
    const bool predict_only = sigma_R_vel == inf && sigma_R_acc == inf;
    if (!std::isfinite(sigma_Q) || sigma_Q < 0.0f) return false;
    if (!predict_only && (!std::isfinite(sigma_R_vel) || !std::isfinite(sigma_R_acc) ||
                          sigma_R_vel <= 0.0f || sigma_R_acc <= 0.0f)) return false;
    if ((!kalman_initialized_ || !predict_only) &&
        (!std::isfinite(body_vel) || !std::isfinite(acc_x))) return false;

    if (!kalman_initialized_) {
        kalman_vel = body_vel;
        kalman_acc = acc_x;
        p_vv_ = predict_only ? 0.0 : static_cast<double>(sigma_R_vel) * sigma_R_vel;
        p_va_ = 0.0;
        p_aa_ = predict_only ? 0.0 : static_cast<double>(sigma_R_acc) * sigma_R_acc;
        kalman_initialized_ = true;
        return true;
    }

    constexpr double dt = 1.0 / WHEEL_CHASSIS_HZ;
    const double q_a = sigma_Q * dt;
    const double q_v = 0.5 * dt * q_a;
    const double v_pred = kalman_vel + dt * kalman_acc;
    const double a_pred = kalman_acc;
    const double p_vv = p_vv_ + dt * (2.0 * p_va_ + dt * p_aa_) + q_v * q_v;
    const double p_va = p_va_ + dt * p_aa_ + q_v * q_a;
    const double p_aa = p_aa_ + q_a * q_a;

    constexpr double max_output = std::numeric_limits<float>::max();
    if (predict_only) {
        if (!std::isfinite(v_pred) || std::abs(v_pred) > max_output ||
            !std::isfinite(p_vv) || !std::isfinite(p_va) || !std::isfinite(p_aa)) return false;
        kalman_vel = static_cast<float>(v_pred);
        p_vv_ = p_vv;
        p_va_ = p_va;
        p_aa_ = p_aa;
        return true;
    }

    const double r_vv = static_cast<double>(sigma_R_vel) * sigma_R_vel;
    const double r_aa = static_cast<double>(sigma_R_acc) * sigma_R_acc;
    const double s_vv = p_vv + r_vv;
    const double s_aa = p_aa + r_aa;
    const double det_p = p_vv * p_aa - p_va * p_va;
    const double det_s = det_p + p_vv * r_aa + r_vv * s_aa;
    if (!std::isfinite(det_s) || det_s <= 0.0) return false;
    const double k_vv = (det_p + p_vv * r_aa) / det_s;
    const double k_va = p_va * r_vv / det_s;
    const double k_av = p_va * r_aa / det_s;
    const double k_aa = (det_p + p_aa * r_vv) / det_s;
    const double dv = body_vel - v_pred;
    const double da = acc_x - a_pred;
    const double v_next = v_pred + k_vv * dv + k_va * da;
    const double a_next = a_pred + k_av * dv + k_aa * da;

    // Joseph 更新，只保存对称协方差的三个独立元素。
    const double m_vv = r_vv * s_aa / det_s;
    const double m_aa = r_aa * s_vv / det_s;
    const double mp_vv = m_vv * p_vv - k_va * p_va;
    const double mp_va = m_vv * p_va - k_va * p_aa;
    const double mp_av = m_aa * p_va - k_av * p_vv;
    const double mp_aa = m_aa * p_aa - k_av * p_va;
    const double next_p_vv = mp_vv * m_vv - mp_va * k_va + k_vv * k_vv * r_vv + k_va * k_va * r_aa;
    const double next_p_va = mp_va * m_aa - mp_vv * k_av + k_vv * k_av * r_vv + k_va * k_aa * r_aa;
    const double next_p_aa = mp_aa * m_aa - mp_av * k_av + k_av * k_av * r_vv + k_aa * k_aa * r_aa;
    if (!std::isfinite(v_next) || !std::isfinite(a_next) ||
        std::abs(v_next) > max_output || std::abs(a_next) > max_output ||
        !std::isfinite(next_p_vv) || !std::isfinite(next_p_va) || !std::isfinite(next_p_aa) ||
        next_p_vv < 0.0 || next_p_aa < 0.0) return false;

    kalman_vel = static_cast<float>(v_next);
    kalman_acc = static_cast<float>(a_next);
    p_vv_ = next_p_vv;
    p_va_ = next_p_va;
    p_aa_ = next_p_aa;
    return true;
}
}
