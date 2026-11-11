//
// Created by KLnormal on 2026/11/9.
//

#include "chassis_coor.h"
#include "chassis_data_conifg.h"
#include <cmath>
#include <numbers>

using namespace Coordinator;

namespace {
constexpr float target_leg_rad = 0.5f * std::numbers::pi_v<float>;
}

void chassis_coor::chassis_observer()
{
    base_data_observer_->chassis_tick();
    base_.update_data(base_data_observer_->get_chassis_data());
    const auto& b = base_.get_data();
    const auto& c = chassis_.get_data();
    left_leg_observer_->leg_tick(b.j4.pos_, b.j3.pos_, b.j1.pos_, b.j2.pos_);
    if (right_leg_observer_ != left_leg_observer_)
        right_leg_observer_->leg_tick(b.j4.pos_, b.j3.pos_, b.j1.pos_, b.j2.pos_);
    const auto left = left_leg_observer_->get_leg_data().first;
    const auto right = right_leg_observer_->get_leg_data().second;
    processed_data next{};
    next.lqr_state.x = (b.motor_left.total_angle+b.motor_right.total_angle)*WHEEL_RADIUS/2;
    next.lqr_state.dot_x = (b.motor_left.angular_velocity+b.motor_left.angular_velocity)*WHEEL_RADIUS/2;
    next.lqr_state.theta = b.body.body_theta;
    next.lqr_state.dot_theta = b.body.body_gyro_pitch;
    next.lqr_state.phi = b.body.body_phi;
    next.lqr_state.dot_phi = b.body.body_gyro_yaw;
    next.left_leg_state = {left.leg_phi, left.leg_phi_dot, left.leg_len, left.leg_len_dot};
    next.right_leg_state = {right.leg_phi, right.leg_phi_dot, right.leg_len, right.leg_len_dot};
    chassis_.update_data(next);
}

void chassis_coor::chassis_command()
{
    if (remote->s_l == 1) static_leg =true;
    else static_leg = false;
    if (remote->s_r == 1 && remote->s_l == 1) lqr_enabled_ = true;
    else lqr_enabled_ = false;
}

void chassis_coor::chassis_protection()
{
}

void chassis_coor::chassis_scheduler()
{
    const auto& b = base_.get_data();
    const auto& c = chassis_.get_data();
    lqr_state_ = lqr_enabled_ ? c.lqr_state : chassis_state{};
    left_cmd_ = {};
    right_cmd_ = {};
    const bool leg_enabled = static_leg || lqr_enabled_;
    left_cmd_.target_len = leg_enabled ? 0.18f : c.left_leg_state.current_len;
    right_cmd_.target_len = leg_enabled ? 0.18f : c.right_leg_state.current_len;
    if (leg_enabled) {
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
    auto& lqr = lqr_ctrl_ ? *lqr_ctrl_ : fixed_lqr_;
    output_.lqr = {};
    if (lqr_enabled_) {
        lqr.lqr_enable(lqr_state_);
        lqr.lqr_clc(0.0f, 0.0f, lqr_state_, left_cmd_.current_len, right_cmd_.current_len);
        output_.lqr = lqr.get_output();
    } else {
        lqr.lqr_disable();
    }
    left_cmd_.forward_tor_M = 0;
    right_cmd_.forward_tor_M = 0;
    left_cmd_.forward_force_L = 0;
    right_cmd_.forward_force_L = 0;
    left_cmd_.forward_y = 30;
    right_cmd_.forward_y = 30;
    left_cmd_.forward_x = 0;
    right_cmd_.forward_x = 0;
    left_leg_ctrl_->leg_ctrl_clc(left_cmd_);
    right_leg_ctrl_->leg_ctrl_clc(right_cmd_);
    output_.left_leg = left_leg_ctrl_->get_output();
    output_.right_leg = right_leg_ctrl_->get_output();
    output_.lqr = lqr.get_output();
}

void chassis_coor::chassis_actuator()
{
    float j1 = 0, j2 = 0, j3 = 0, j4 = 0;
    if (static_leg || lqr_enabled_) {
        j1 = output_.right_leg.len_ctrl_small + output_.right_leg.rad_ctrl_small + output_.right_leg.forward_cod_small + output_.right_leg.forward_poly_small;
        j2 = output_.right_leg.len_ctrl_big + output_.right_leg.rad_ctrl_big + output_.right_leg.forward_cod_big + output_.right_leg.forward_poly_big;
        j3 = output_.left_leg.len_ctrl_big + output_.left_leg.rad_ctrl_big + output_.left_leg.forward_cod_big + output_.left_leg.forward_poly_big;
        j4 = output_.left_leg.len_ctrl_small + output_.left_leg.rad_ctrl_small + output_.left_leg.forward_cod_small + output_.left_leg.forward_poly_small;
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
        if (lqr_enabled_) {
            // 按原装力矩常数和当前减速比，将轮轴 N·m 换算为原始电流指令。
            constexpr float rotor_kt = WHEEL_ORIGINAL_TORQUE_CONSTANT / WHEEL_ORIGINAL_REDUCTION;
            constexpr float torque_to_current = WHEEL_MAX_RAW_CURRENT / (WHEEL_MAX_CURRENT * rotor_kt * WHEEL_REDUCTION);
            auto c = chassis_.get_data();
            auto ins =ins::data();
            bsp_uart_printf(E_UART_1,"%f,%f,%f,%f,%f\r\n",
                output_.lqr.T_w_l * LEFT_MOTOR_DIR * torque_to_current,
                output_.lqr.T_w_r * LEFT_MOTOR_DIR * torque_to_current,
                c.lqr_state.theta/PI*180.0f,
                c.lqr_state.dot_theta,
                c.lqr_state.x
                );

            motors_.left->update(output_.lqr.T_w_l * LEFT_MOTOR_DIR * torque_to_current);
            motors_.right->update(output_.lqr.T_w_r * RIGHT_MOTOR_DIR * torque_to_current);
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
