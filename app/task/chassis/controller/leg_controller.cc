//
// Created by KLnormal on 2026/11/7.
//

#include "leg_controller.h"
#include "chassis_data_conifg.h"

using namespace Controller;

LegController::LegController():LegController({}, {}, {}, {}) {}

void LegController::leg_ctrl_clc(leg_ctrl_data ctrl)
{
    leg_ctrl_ = ctrl;
    output_ = {};
    if (!ctrl_flag) return;
    leg_forward_clc();
    leg_pos_ctrl();
}

void LegController::leg_forward_clc()
{
    const auto cod = vmc_co_clc(leg_ctrl_.small_rad, leg_ctrl_.big_rad,
        leg_ctrl_.forward_x, leg_ctrl_.forward_y, leg_ctrl_.current_rad);
    output_.forward_cod_small = cod.first;
    output_.forward_cod_big = cod.second;
    const auto pol = vmc_pol_clc(leg_ctrl_.small_rad, leg_ctrl_.big_rad,
        leg_ctrl_.forward_tor_M, leg_ctrl_.forward_force_L);
    output_.forward_poly_small = pol.first;
    output_.forward_poly_big = pol.second;
}

void LegController::leg_pos_ctrl()
{
    constexpr float dt = 1.0f / WHEEL_CHASSIS_HZ;
    const float target_speed = len_pid.update(leg_ctrl_.current_len, leg_ctrl_.target_len, dt);
    const float force_L = speed_pid.update(leg_ctrl_.current_speed, target_speed, dt);
    const float target_omega = rad_pid.update(0.0f, leg_ctrl_.rad_error, dt);
    const float tor_M = omega_pid.update(leg_ctrl_.current_omega, target_omega, dt);
    const auto len_tor = vmc_pol_clc(leg_ctrl_.small_rad, leg_ctrl_.big_rad, 0.0f, force_L);
    const auto rad_tor = vmc_pol_clc(leg_ctrl_.small_rad, leg_ctrl_.big_rad, tor_M, 0.0f);
    output_.len_ctrl_small = len_tor.first;
    output_.len_ctrl_big = len_tor.second;
    output_.rad_ctrl_small = rad_tor.first;
    output_.rad_ctrl_big = rad_tor.second;
}

void LegController::leg_clear()
{
    len_pid.clear();
    speed_pid.clear();
    rad_pid.clear();
    omega_pid.clear();
    leg_ctrl_ = {};
    output_ = {};
}

void LegController::leg_disable()
{
    ctrl_flag = false;
    leg_clear();
}

void LegController::leg_enable()
{
    if (ctrl_flag) return;
    leg_clear();
    ctrl_flag = true;
}
