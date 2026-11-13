//
// Created by KLnormal on 2026/11/9.
//

#include "chassis_coor.h"
#include "chassis_data_conifg.h"
#include <algorithm>
#include <cmath>
#include <numbers>

using namespace Coordinator;

namespace {
constexpr float target_leg_rad = 0.5f * std::numbers::pi_v<float>;

float support_scale(float theta) {
    // 相对竖直方向的偏角：30° 内为 1，30°～40° 以三次曲线降到 0。
    constexpr float fade_start = std::numbers::pi_v<float> / 6.0f;
    constexpr float fade_width = std::numbers::pi_v<float> / 18.0f;
    const float t = std::clamp((fabsf(theta) - fade_start) / fade_width, 0.0f, 1.0f);
    return (1.0f - t) * (1.0f - t) * (1.0f + 2.0f * t);
}
}

void chassis_coor::chassis_observer()
{
    //更新电机和IMU信息
    base_data_observer_->chassis_tick();
    base_.update_data(base_data_observer_->get_chassis_data());
    const auto& b = base_.get_data();

    //腿部数据处理
    left_leg_observer_->leg_tick(b.j4.pos_, b.j3.pos_, b.j1.pos_, b.j2.pos_);
    if (right_leg_observer_ != left_leg_observer_)
        right_leg_observer_->leg_tick(b.j4.pos_, b.j3.pos_, b.j1.pos_, b.j2.pos_);
    const auto left = left_leg_observer_->get_leg_data().first;
    const auto right = right_leg_observer_->get_leg_data().second;
    processed_data next{};

    //腿控制和LQR控制所需要用到的状态均存在chassis_变量中
    //更新LQR状态
    next.lqr_state.x = (b.motor_left.total_angle+b.motor_right.total_angle)*WHEEL_RADIUS/2;
    next.lqr_state.dot_x = (b.motor_left.linear_velocity + b.motor_right.linear_velocity)/2;//这个是线速度！
    next.lqr_state.theta = b.body.body_theta;
    next.lqr_state.dot_theta = b.body.body_gyro_pitch;
    next.lqr_state.left_theta = left.leg_phi-PI/2;
    next.lqr_state.dot_left_theta = left.leg_phi_dot;
    next.lqr_state.right_theta = right.leg_phi-PI/2;
    next.lqr_state.dot_right_theta = right.leg_phi_dot;
    next.lqr_state.phi = b.body.body_phi;
    next.lqr_state.dot_phi = b.body.body_gyro_yaw;

    //更新腿状态
    next.left_leg_state = {left.leg_phi, left.leg_phi_dot, left.leg_len, left.leg_len_dot};
    next.right_leg_state = {right.leg_phi, right.leg_phi_dot, right.leg_len, right.leg_len_dot};
    chassis_.update_data(next);
}

void chassis_coor::chassis_command()
{
    static_leg = remote->s_l == 1;

    if (static_leg && remote->s_r == 0) {
        lqr_mode_ = lqr_mode::bench_control;
    }
    else if (static_leg && remote->s_r == 1)
    {
        lqr_mode_ = lqr_mode::full_control;
    }
    else
    {
        lqr_mode_ = lqr_mode::rest;
    }

    //遥感目标控制命令
    target_vel_ = 2*std::clamp(remote->rc_l[1] / 800.0f, -1.0f, 1.0f);
    target_omega_ = -5.0f * std::clamp(remote->reserved / 660.0f, -1.0f, 1.0f);
    target_leg_vel = 0.3f*std::clamp(remote->rc_r[1] / 800.0f, -1.0f, 1.0f);
}

void chassis_coor::chassis_protection()
{
}

void chassis_coor::chassis_scheduler()
{
    const auto& b = base_.get_data();
    const auto& c = chassis_.get_data();
    lqr_state_ = c.lqr_state;
    left_cmd_ = {};
    right_cmd_ = {};

    //腿长控制计算
    const bool leg_enabled = static_leg;
    constexpr float dt = 1.0f / WHEEL_CHASSIS_HZ;
    if (leg_enabled) {
        target_height_ = std::clamp(target_height_ + target_leg_vel * dt, 0.14f, 0.33f);
    } else {
        target_height_ = 0.18f;
    }
    if (leg_enabled)
    {
        float left_theta = c.lqr_state.left_theta;
        float right_theta = c.lqr_state.right_theta;
        left_cmd_.target_len = fabsf(left_theta) < 30.0f/180.0f*PI ? (target_height_/cosf(left_theta))      : target_height_;
        right_cmd_.target_len = fabsf(right_theta) < 30.0f/180.0f*PI ? (target_height_/cosf(right_theta))   : target_height_;
    }
    else
    {
        left_cmd_.target_len = c.left_leg_state.current_len;
        right_cmd_.target_len = c.right_leg_state.current_len;
    }
    if (leg_enabled && lqr_mode_ != lqr_mode::full_control) {
        const float left_error = target_leg_rad - c.left_leg_state.current_rad;
        const float right_error = target_leg_rad - c.right_leg_state.current_rad;
        left_cmd_.rad_error = atan2f(sinf(left_error), cosf(left_error));
        right_cmd_.rad_error = atan2f(sinf(right_error), cosf(right_error));
    }
    left_cmd_.current_len = c.left_leg_state.current_len;
    left_cmd_.current_speed = c.left_leg_state.current_speed;
    left_cmd_.current_rad = c.left_leg_state.current_rad;
    left_cmd_.current_omega = c.left_leg_state.current_omega;
    left_cmd_.small_rad = b.j4.pos_;
    left_cmd_.big_rad = b.j3.pos_;
    right_cmd_.current_len = c.right_leg_state.current_len;
    right_cmd_.current_speed = c.right_leg_state.current_speed;
    right_cmd_.current_rad = c.right_leg_state.current_rad;
    right_cmd_.current_omega = c.right_leg_state.current_omega;
    right_cmd_.small_rad = b.j1.pos_;
    right_cmd_.big_rad = b.j2.pos_;
}

void chassis_coor::chassis_controller()
{
    left_leg_ctrl_->leg_enable();
    right_leg_ctrl_->leg_enable();
    const bool full_control = lqr_mode_ == lqr_mode::full_control;
    output_.lqr = {};
    if (lqr_ctrl_ && lqr_mode_ != lqr_mode::rest) {
        lqr_ctrl_->lqr_enable(lqr_state_);
        if (full_control) {
            auto l = chassis_.get_data();
            lqr_ctrl_->lqr_clc_fitted(target_vel_, target_omega_, lqr_state_,
                l.left_leg_state.current_len, l.right_leg_state.current_len);
        } else {
            lqr_ctrl_->lqr_clc_bench(target_vel_, target_omega_, lqr_state_, static_matrix_K);
        }
        output_.lqr = lqr_ctrl_->get_output();
    } else if (lqr_ctrl_) {
        lqr_ctrl_->lqr_disable();
    }
    constexpr float support_force_y = CAR_WEIGHT / 4.0f * 9.8f;
    const float left_support_force = support_force_y * support_scale(lqr_state_.left_theta);
    const float right_support_force = support_force_y * support_scale(lqr_state_.right_theta);
    // left_cmd_.forward_tor_M = output_.lqr.forward_T_bl;
    // right_cmd_.forward_tor_M = output_.lqr.forward_T_br;
    left_cmd_.forward_tor_M  = 0;
    right_cmd_.forward_tor_M = 0;
    left_cmd_.forward_force_L = full_control ? left_support_force * sinf(left_cmd_.current_rad) : 0.0f;
    right_cmd_.forward_force_L = full_control ? right_support_force * sinf(right_cmd_.current_rad) : 0.0f;
    left_cmd_.forward_y = full_control ? 0.0f : left_support_force;
    right_cmd_.forward_y = full_control ? 0.0f : right_support_force;
    left_cmd_.forward_x = 0;
    right_cmd_.forward_x = 0;
    left_leg_ctrl_->leg_ctrl_clc(left_cmd_, true, !full_control);
    right_leg_ctrl_->leg_ctrl_clc(right_cmd_, true, !full_control);
    output_.left_leg = left_leg_ctrl_->get_output();
    output_.right_leg = right_leg_ctrl_->get_output();
}

void chassis_coor::chassis_actuator()
{
    const bool full_control = lqr_mode_ == lqr_mode::full_control;
    float j1 = 0, j2 = 0, j3 = 0, j4 = 0;
    if (static_leg) {
        j1 = output_.right_leg.len_ctrl_small + output_.right_leg.rad_ctrl_small
            + output_.right_leg.forward_cod_small + output_.right_leg.forward_poly_small;
        j2 = output_.right_leg.len_ctrl_big + output_.right_leg.rad_ctrl_big
            + output_.right_leg.forward_cod_big + output_.right_leg.forward_poly_big;
        j3 = output_.left_leg.len_ctrl_big + output_.left_leg.rad_ctrl_big +
            output_.left_leg.forward_cod_big + output_.left_leg.forward_poly_big;
        j4 = output_.left_leg.len_ctrl_small + output_.left_leg.rad_ctrl_small +
            output_.left_leg.forward_cod_small + output_.left_leg.forward_poly_small;
        if (lqr_ctrl_ && full_control) {
            const auto [left_small, left_big] = vmc_pol_clc(
                left_cmd_.small_rad, left_cmd_.big_rad, output_.lqr.T_b_l, 0.0f);
            const auto [right_small, right_big] = vmc_pol_clc(
                right_cmd_.small_rad, right_cmd_.big_rad, output_.lqr.T_b_r, 0.0f);
            j1 += right_small;
            j2 += right_big;
            j3 += left_big;
            j4 += left_small;
        }
    }
    if (remote->s_l == 1)
    {
        motors_.right_forward->control(0, 0, 0, 0, j1 * J1_dir);
        motors_.right_back->control(0, 0, 0, 0, j2 * J2_dir);
        motors_.left_back->control(0, 0, 0, 0, j3 * J3_dir);
        motors_.left_forward->control(0, 0, 0, 0, j4 * J4_dir);
        // motors_.right_forward->control(0, 0, 0, 0, 0);
        // motors_.right_back->control(0, 0, 0, 0, 0);
        // motors_.left_back->control(0, 0, 0, 0, 0);
        // motors_.left_forward->control(0, 0, 0, 0, 0);
        if (lqr_mode_!= lqr_mode::rest) {
            const auto& c = chassis_.get_data();
            const auto& b = base_.get_data();
            bsp_uart_printf(E_UART_1,"%f,%f,%f,%f,%f\r\n",
                c.lqr_state.x,
                c.left_leg_state.current_len,
                output_.lqr.forward_T_br,
                output_.lqr.forward_T_wl,
                output_.lqr.forward_T_wr
                );
        }
        if (lqr_ctrl_ && lqr_mode_ != lqr_mode::rest) {
            // 按原装力矩常数和当前减速比，将轮轴 N·m 换算为原始电流指令。
            constexpr float rotor_kt = WHEEL_ORIGINAL_TORQUE_CONSTANT / WHEEL_ORIGINAL_REDUCTION;
            constexpr float torque_to_current = WHEEL_MAX_RAW_CURRENT / (WHEEL_MAX_CURRENT * rotor_kt * WHEEL_REDUCTION);
            // motors_.left->update((output_.lqr.T_w_l+output_.lqr.forward_T_wl) * LEFT_MOTOR_DIR * torque_to_current);
            // motors_.right->update((output_.lqr.T_w_r+output_.lqr.forward_T_wr) * RIGHT_MOTOR_DIR * torque_to_current);
            motors_.left->update((output_.lqr.T_w_l) * LEFT_MOTOR_DIR * torque_to_current);
            motors_.right->update((output_.lqr.T_w_r) * RIGHT_MOTOR_DIR * torque_to_current);
            // motors_.left->update(0);
            // motors_.right->update(0);
        } else {
            motors_.left->update(0);
            motors_.right->update(0);
        }
    }
    else
    {
        motors_.right_forward->control(0, 0, 0, 0, 0);
        motors_.right_back->control(0, 0, 0, 0, 0);
        motors_.left_back->control(0, 0, 0, 0, 0);
        motors_.left_forward->control(0, 0, 0, 0, 0);
        motors_.left->update(0);
        motors_.right->update(0);
    }
}
