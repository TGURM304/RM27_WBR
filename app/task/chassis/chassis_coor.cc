//
// Created by KLnormal on 2026/11/9.
//

#include "chassis_coor.h"
#include "chassis_data_conifg.h"
#include <cmath>
#include <numbers>

using namespace Coordinator;

void chassis_coor::chassis_observer()
{
    base_data_observer_->chassis_tick();
    base_.update_data(base_data_observer_->get_chassis_data());
    const auto& b = base_.get_data();
    left_leg_observer_->leg_tick(b.j4.pos_, b.j3.pos_, b.j1.pos_, b.j2.pos_);
    if (right_leg_observer_ != left_leg_observer_)
        right_leg_observer_->leg_tick(b.j4.pos_, b.j3.pos_, b.j1.pos_, b.j2.pos_);
    const auto left = left_leg_observer_->get_leg_data().first;
    const auto right = right_leg_observer_->get_leg_data().second;
    processed_data next{};
    next.lqr_state.theta = b.body.body_theta;
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
    lqr_state_ = {};
    if (lqr_enabled_) lqr_state_.theta = c.lqr_state.theta;
    left_cmd_ = {};
    right_cmd_ = {};
    const bool leg_enabled = static_leg || lqr_enabled_;
    left_cmd_.target_len = leg_enabled ? 0.18f : c.left_leg_state.current_len;
    right_cmd_.target_len = leg_enabled ? 0.18f : c.right_leg_state.current_len;
    if (leg_enabled) {
        constexpr float target_rad = 0.5f * std::numbers::pi_v<float>;
        const float left_error = target_rad - c.left_leg_state.current_rad;
        const float right_error = target_rad - c.right_leg_state.current_rad;
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
    lqr_output_ = {};
    if (lqr_enabled_) {
        lqr_ctrl_->lqr_enable();
        lqr_ctrl_->lqr_clc(0.0f, 0.0f, lqr_state_, left_cmd_.current_len, right_cmd_.current_len);
        lqr_output_ = lqr_ctrl_->get_output();
    } else if (lqr_ctrl_) {
        lqr_ctrl_->lqr_disable();
    }
    left_cmd_.forward_tor_M = lqr_enabled_ ? lqr_output_.T_b_l : 0.0f;
    right_cmd_.forward_tor_M = lqr_enabled_ ? lqr_output_.T_b_r : 0.0f;
    left_leg_ctrl_->leg_ctrl_clc(left_cmd_);
    right_leg_ctrl_->leg_ctrl_clc(right_cmd_);
    left_output_ = left_leg_ctrl_->get_output();
    right_output_ = right_leg_ctrl_->get_output();
}

void chassis_coor::chassis_actuator()
{
    float j1 = 0, j2 = 0, j3 = 0, j4 = 0;
    // float left_wheel = 0, right_wheel = 0;
    if (static_leg || lqr_enabled_) {
        j1 = right_output_.len_ctrl_small + right_output_.rad_ctrl_small + right_output_.forward_cod_small + right_output_.forward_poly_small;
        j2 = right_output_.len_ctrl_big + right_output_.rad_ctrl_big + right_output_.forward_cod_big + right_output_.forward_poly_big;
        j3 = left_output_.len_ctrl_big + left_output_.rad_ctrl_big + left_output_.forward_cod_big + left_output_.forward_poly_big;
        j4 = left_output_.len_ctrl_small + left_output_.rad_ctrl_small + left_output_.forward_cod_small + left_output_.forward_poly_small;
    }
    if (lqr_enabled_) {
        // left_wheel = lqr_output_.T_w_l;
        // right_wheel = lqr_output_.T_w_r;
    }
    if (remote->s_l == 1)
    {
        motors_.right_forward->control(0, 0, 0, 0, j1 * J1_dir);
        motors_.right_back->control(0, 0, 0, 0, j2 * J2_dir);
        motors_.left_back->control(0, 0, 0, 0, j3 * J3_dir);
        motors_.left_forward->control(0, 0, 0, 0, j4 * J4_dir);
        if (lqr_enabled_) {
            // 按原装力矩常数和当前减速比，将轮轴 N·m 换算为原始电流指令。
            // float raw_current = (1/WHEEL_ORIGINAL_TORQUE_CONSTANT)*WHEEL_MAX_RAW_CURRENT/WHEEL_MAX_CURRENT;
            // float percent = WHEEL_REDUCTION/WHEEL_ORIGINAL_REDUCTION;
            motors_.left->update(1000);
            motors_.right->update(-1000);
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
