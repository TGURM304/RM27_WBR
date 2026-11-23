#include "chassis_vel_observer.h"

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
    data_ = {};
    initialized_ = false;
    acc_filter_.clear();
}

void ChassisVelObserver::kalman_clc(float raw_acc, float raw_left, float raw_right,
                                   float q_vv, float q_aa, float r_vv, float r_aa)
{
    if (!enabled_) return;
    data_.raw_data = {acc_filter_.update(raw_acc), raw_left, raw_right};
    const double velocity = 0.5 * (static_cast<double>(raw_left) + raw_right);
    const double acceleration = data_.raw_data.acc_x;
    if (!initialized_) {
        data_.kalman_vel = static_cast<float>(velocity);
        data_.kalman_acc = static_cast<float>(acceleration);
        data_.p_vv = data_.p_aa = 1.0;
        data_.p_va = 0.0;
        initialized_ = true;
        return;
    }

    constexpr double dt = 1.0 / 1000.0;
    const double v_pred = data_.kalman_vel + dt * data_.kalman_acc;
    const double a_pred = data_.kalman_acc;
    const double p_vv = data_.p_vv + dt * (2.0 * data_.p_va + dt * data_.p_aa) + q_vv;
    const double p_va = data_.p_va + dt * data_.p_aa;
    const double p_aa = data_.p_aa + q_aa;
    const double s_vv = p_vv + r_vv;
    const double s_aa = p_aa + r_aa;
    const double det_p = p_vv * p_aa - p_va * p_va;
    const double det_s = det_p + p_vv * r_aa + r_vv * s_aa;
    const double k_vv = (det_p + p_vv * r_aa) / det_s;
    const double k_va = p_va * r_vv / det_s;
    const double k_av = p_va * r_aa / det_s;
    const double k_aa = (det_p + p_aa * r_vv) / det_s;
    const double dv = velocity - v_pred;
    const double da = acceleration - a_pred;
    data_.kalman_vel = static_cast<float>(v_pred + k_vv * dv + k_va * da);
    data_.kalman_acc = static_cast<float>(a_pred + k_av * dv + k_aa * da);

    // Joseph 形式更新 P，复用对称矩阵的三个元素。
    const double m_vv = r_vv * s_aa / det_s;
    const double m_aa = r_aa * s_vv / det_s;
    const double mp_vv = m_vv * p_vv - k_va * p_va;
    const double mp_va = m_vv * p_va - k_va * p_aa;
    const double mp_av = m_aa * p_va - k_av * p_vv;
    const double mp_aa = m_aa * p_aa - k_av * p_va;
    data_.p_vv = mp_vv * m_vv - mp_va * k_va + k_vv * k_vv * r_vv + k_va * k_va * r_aa;
    data_.p_va = mp_va * m_aa - mp_vv * k_av + k_vv * k_av * r_vv + k_va * k_aa * r_aa;
    data_.p_aa = mp_aa * m_aa - mp_av * k_av + k_av * k_av * r_vv + k_aa * k_aa * r_aa;
}
}
