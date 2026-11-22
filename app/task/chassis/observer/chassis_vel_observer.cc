//
// Created by KLnormal on 2026/11/13.
//

#include "chassis_vel_observer.h"

#include "chassis_data_conifg.h"
#include "math/matrix.h"
#include <cmath>
#include <limits>

namespace
{
float wheel_mount_rate(const Observer::leg_data& leg, const Observer::joints& big_joint)
{
    const float sin_phi = std::sin(leg.leg_phi);
    const float cos_phi = std::cos(leg.leg_phi);
    const float ag_x = LEG_LEN_AG * std::cos(big_joint.pos_);
    const float ag_y = LEG_LEN_AG * std::sin(big_joint.pos_);
    const float gw_x = leg.leg_len * cos_phi - ag_x;
    const float gw_y = leg.leg_len * sin_phi - ag_y;

    // GW = AW − AG，角速度为 (GW × dGW/dt) / |GW|²。
    const float tangent_speed = leg.leg_len * leg.leg_phi_dot;
    const float gw_vx = leg.leg_len_dot * cos_phi - tangent_speed * sin_phi + ag_y * big_joint.vel_;
    const float gw_vy = leg.leg_len_dot * sin_phi + tangent_speed * cos_phi - ag_x * big_joint.vel_;
    return (gw_x * gw_vy - gw_y * gw_vx) / (LEG_LEN_GW * LEG_LEN_GW);
}

float wheel_relative_velocity(const Observer::leg_data& leg, const Observer::body_posture& posture,
                              const math::matrix<3, 3>& body_to_level)
{
    const float cos_phi = std::cos(leg.leg_phi), sin_phi = std::sin(leg.leg_phi);
    const float x = leg.leg_len * cos_phi, z = -leg.leg_len * sin_phi;
    const float tangent_speed = leg.leg_len * leg.leg_phi_dot;

    // 机体 X 向前、Z 向上。左右安装点的横向偏置在轮轴中心平均时相消。
    const math::matrix<3, 1> relative_velocity{
        leg.leg_len_dot * cos_phi - tangent_speed * sin_phi + posture.body_gyro_pitch * z,
        posture.body_gyro_yaw * x - posture.body_gyro_roll * z,
        -leg.leg_len_dot * sin_phi - tangent_speed * cos_phi - posture.body_gyro_pitch * x
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
    diagnostics_ = {};
}

float ChassisVelObserver::update(Base_data_observer *base_observer, Leg_observer *left_observer,
                                 Leg_observer *right_observer, float slip_confidence)
{
    diagnostics_ = {};
    if (!enabled_) return 0.0f;
    const float body_vel = chassis_move_clc(base_observer, left_observer, right_observer);
    const bool slip_adapted = slip_confidence > SLIP_KF_CONFIDENCE_GATE;
    const double q_vv = slip_adapted ? SLIP_KF_Q_VEL : CHASSIS_KF_Q_VEL;
    const double q_aa = slip_adapted ? SLIP_KF_Q_ACC : CHASSIS_KF_Q_ACC;
    const double r_vv = slip_adapted ? SLIP_KF_R_VEL : CHASSIS_KF_R_VEL;
    const double r_aa = slip_adapted ? SLIP_KF_R_ACC : CHASSIS_KF_R_ACC;

    //测试用代码
    // const double q_vv = 1.0f/1000.0f;
    // const double q_aa = 10.0f/1000.0f;
    // const double r_vv = 0.1*100000;
    // const double r_aa = 0.1f;

    diagnostics_.predicted_velocity = static_cast<float>(kalman_vel + (1.0 / WHEEL_CHASSIS_HZ) * kalman_acc);
    const bool had_prediction = kalman_initialized_;
    if (chassis_kalman(diagnostics_.acc_x, body_vel, q_vv, q_aa, r_vv, r_aa)
        && std::isfinite(wheel_relative_vel_)) {
        diagnostics_.valid = had_prediction && std::isfinite(diagnostics_.left_velocity)
                          && std::isfinite(diagnostics_.right_velocity) && std::isfinite(body_vel)
                          && std::isfinite(diagnostics_.acc_x) && std::isfinite(diagnostics_.yaw_rate)
                          && std::isfinite(diagnostics_.predicted_velocity);
    }
    return body_vel;
}

float ChassisVelObserver::chassis_move_clc(Base_data_observer *base_observer, Leg_observer *left_observer,
                                           Leg_observer *right_observer)
{
    const auto chassis = base_observer->get_chassis_data();
    const auto left_leg = left_observer->get_leg_data().first;
    const auto right_leg = right_observer->get_leg_data().second;
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
    const float relative_left = wheel_relative_velocity(left_leg, posture, body_to_level);
    const float relative_right = wheel_relative_velocity(right_leg, posture, body_to_level);
    const float relative_vel = 0.5f * (relative_left + relative_right);
    const float body_vel = vel_wheel - relative_vel;
    // 横向位置 y 的前向速度为 -y * yaw_rate，与腿运动补偿使用同一投影。
    const float yaw_rate = body_to_level(0, 0) * posture.body_gyro_yaw
                         - body_to_level(0, 2) * posture.body_gyro_roll;
    wheel_relative_vel_ = relative_vel;
    diagnostics_ = {vel_wheel_left - relative_left, vel_wheel_right - relative_right,
                    body_vel, acc_x, yaw_rate, 0.0f, false};
    return body_vel;
}

bool ChassisVelObserver::chassis_kalman(float acc_x, float body_vel,
                                      double q_vv, double q_aa, double r_vv, double r_aa)
{
    if (!std::isfinite(body_vel) || !std::isfinite(acc_x)) return false;

    if (!kalman_initialized_) {
        kalman_vel = body_vel;
        kalman_acc = acc_x;
        p_vv_ = 1.0;
        p_va_ = 0.0;
        p_aa_ = 1.0; // 初始 P=I，与参考工程一致。
        kalman_initialized_ = true;
        return true;
    }

    constexpr double dt = 1.0 / WHEEL_CHASSIS_HZ;
    const double v_pred = kalman_vel + dt * kalman_acc;
    const double a_pred = kalman_acc;
    const double p_vv = p_vv_ + dt * (2.0 * p_va_ + dt * p_aa_) + q_vv;
    const double p_va = p_va_ + dt * p_aa_;
    const double p_aa = p_aa_ + q_aa;

    constexpr double max_output = std::numeric_limits<float>::max();
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
