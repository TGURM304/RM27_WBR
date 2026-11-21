//
// Created by KLnormal on 2026/11/9.
//

#include "chassis_coor.h"
#include "chassis_data_conifg.h"
#include "bsp/time.h"
#include <algorithm>
#include <cmath>

using namespace Coordinator;

namespace {
float wrap_angle(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

float support_scale(float theta) {
    const float t = std::clamp((fabsf(theta) - LEG_SUPPORT_FADE_START) /
                              (LEG_SUPPORT_FADE_END - LEG_SUPPORT_FADE_START), 0.0f, 1.0f);
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
    next.lqr_state.dot_x = 0.5f * (b.motor_left.linear_velocity + b.motor_right.linear_velocity);
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
    remote_ = rc::dr16::state();
    target_vel_ = 2.0f * std::clamp(remote_.rc_l[1] / 800.0f, -1.0f, 1.0f);
    target_omega_ = -5.0f * std::clamp(remote_.reserved / 660.0f, -1.0f, 1.0f);
    target_leg_vel_ = 0.3f * std::clamp(remote_.rc_r[1] / 800.0f, -1.0f, 1.0f);
    start_request_ = remote_seen_ && remote_.timestamp != 0 && remote_.s_l == 1 && !last_enabled_;
    if (remote_.timestamp != 0) {
        remote_seen_ = true;
        last_enabled_ = remote_.s_l == 1;
    }
}

void chassis_coor::chassis_condition()
{
    const auto& b = base_.get_data();
    const auto& c = chassis_.get_data();
    const float left_angle = wrap_angle(c.left_leg_state.current_rad - STAND_LEG_RAD);
    const float right_angle = wrap_angle(c.right_leg_state.current_rad - STAND_LEG_RAD);
    const bool motors_safe = b.j1.state == 1 && b.j2.state == 1 && b.j3.state == 1 && b.j4.state == 1;
    const bool posture_safe = fabsf(left_angle) <= BODY_LEG_DEG_SAFE && fabsf(right_angle) <= BODY_LEG_DEG_SAFE &&
                              fabsf(b.body.body_theta) <= BODY_PITCH_SAFE;
    auto next = fsm_.ctrl;
    switch (fsm_.ctrl) {
        case E_REST:
            if (start_request_ && motors_safe) next = E_STAND;
            break;
        case E_STAND:
        case E_NORMAL:
            // LQR 接管后延迟启用倾角保护，电机状态始终检查。
            if (!motors_safe || (fsm_.ctrl == E_NORMAL &&
                static_cast<uint32_t>(bsp_time_get_ms() - normal_start_ms_) >= LQR_ANGLE_GUARD_DELAY_MS &&
                !posture_safe)) {
                next = E_ERROR;
            } else if (remote_.s_l != 1) {
                next = E_REST;
            } else if (fsm_.ctrl == E_STAND) {
                const bool ready =
                    c.left_leg_state.current_len >= STAND_LEN_MIN && c.left_leg_state.current_len <= STAND_LEN_MAX &&
                    c.right_leg_state.current_len >= STAND_LEN_MIN && c.right_leg_state.current_len <= STAND_LEN_MAX &&
                    fabsf(left_angle) <= STAND_LEG_RAD_TOL && fabsf(right_angle) <= STAND_LEG_RAD_TOL;
                if (ready) next = E_NORMAL;
            }
            break;
        case E_STEP:
        case E_JUMP:
            if (remote_.s_l != 1) next = E_REST;
            break;
        case E_ERROR:
            break;
    }
    if (next != fsm_.ctrl) {
        fsm_.ctrl = next;
        state_entry_ = true;
        restart_pending_ = next == E_ERROR;
        normal_start_ms_ = next == E_NORMAL ? bsp_time_get_ms() : 0;
    }
}

void chassis_coor::chassis_scheduler()
{
    const auto& b = base_.get_data();
    const auto& c = chassis_.get_data();
    lqr_state_ = c.lqr_state;
    left_cmd_ = {};
    right_cmd_ = {};
    output_ = {};
    if (fsm_.ctrl != E_NORMAL) target_vel_ = target_omega_ = target_leg_vel_ = 0.0f;

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

    switch (fsm_.ctrl) {
        case E_STAND: getup(); break;
        case E_NORMAL: base_chassis_ctrl(); break;
        case E_STEP: step_up(); break;
        case E_JUMP: jump(); break;
        case E_REST:
        case E_ERROR: rest(); break;
    }
    state_entry_ = false;
}

void chassis_coor::getup()
{
    left_cmd_.target_len = right_cmd_.target_len = STAND_LEG_LEN;
    left_cmd_.rad_error = wrap_angle(STAND_LEG_RAD - left_cmd_.current_rad);
    right_cmd_.rad_error = wrap_angle(STAND_LEG_RAD - right_cmd_.current_rad);
}

void chassis_coor::base_chassis_ctrl()
{
    if (state_entry_) {
        target_height_ = NORMAL_INIT_HEIGHT;
        left_leg_ctrl_->leg_clear();
        right_leg_ctrl_->leg_clear();
        if (lqr_ctrl_) lqr_ctrl_->lqr_set_zero(lqr_state_);
    } else {
        constexpr float dt = 1.0f / WHEEL_CHASSIS_HZ;
        target_height_ = std::clamp(target_height_ + target_leg_vel_ * dt, STAND_LEG_LEN, NORMAL_HEIGHT_MAX);
    }
    left_cmd_.target_len = fabsf(lqr_state_.left_theta) < NORMAL_HEIGHT_COMP_ANGLE ?
                           target_height_ / cosf(lqr_state_.left_theta) : target_height_;
    right_cmd_.target_len = fabsf(lqr_state_.right_theta) < NORMAL_HEIGHT_COMP_ANGLE ?
                            target_height_ / cosf(lqr_state_.right_theta) : target_height_;
}

void chassis_coor::rest()
{
    if (!state_entry_) return;
    left_leg_ctrl_->leg_disable();
    right_leg_ctrl_->leg_disable();
    if (lqr_ctrl_) lqr_ctrl_->lqr_disable();
    kalman_vel_observer_.clear();
    base_data_observer_->reset_wheel_odometry();
    left_leg_observer_->leg_clear();
    if (right_leg_observer_ != left_leg_observer_) right_leg_observer_->leg_clear();
    target_height_ = NORMAL_INIT_HEIGHT;
}

void chassis_coor::jump() {}
void chassis_coor::step_up() {}

void chassis_coor::restart()
{
    motor::dm *joints[] = {motors_.right_forward, motors_.right_back, motors_.left_back, motors_.left_forward};
    for (auto *joint : joints) {
        if (joint->state().err == 1) continue;
        joint->reset();
        joint->enable();
    }
    motor::dji *wheels[] = {motors_.left, motors_.right};
    for (auto *wheel : wheels) {
        if (wheel->enabled) continue;
        wheel->clear();
        wheel->enable();
    }
}

void chassis_coor::chassis_controller()
{
    output_ = {};
    const bool normal = fsm_.ctrl == E_NORMAL;
    if (!normal && fsm_.ctrl != E_STAND) {
        left_leg_ctrl_->leg_disable();
        right_leg_ctrl_->leg_disable();
        if (lqr_ctrl_) lqr_ctrl_->lqr_disable();
        return;
    }
    left_leg_ctrl_->leg_enable();
    right_leg_ctrl_->leg_enable();
    if (normal) {
        if (lqr_ctrl_) {
            lqr_ctrl_->lqr_enable(lqr_state_);
            lqr_ctrl_->lqr_clc_fitted(target_vel_, target_omega_, lqr_state_,
                                     left_cmd_.current_len, right_cmd_.current_len);
            output_.lqr = lqr_ctrl_->get_output();
        }
        constexpr float support_force_y = CAR_WEIGHT / 4.0f * 9.8f;
        left_cmd_.forward_force_L = support_force_y * support_scale(lqr_state_.left_theta) * sinf(left_cmd_.current_rad);
        right_cmd_.forward_force_L = support_force_y * support_scale(lqr_state_.right_theta) * sinf(right_cmd_.current_rad);
    } else if (lqr_ctrl_) {
        lqr_ctrl_->lqr_disable();
    }
    left_leg_ctrl_->leg_ctrl_clc(left_cmd_, true, !normal);
    right_leg_ctrl_->leg_ctrl_clc(right_cmd_, true, !normal);
    output_.left_leg = left_leg_ctrl_->get_output();
    output_.right_leg = right_leg_ctrl_->get_output();
}

void chassis_coor::chassis_actuator()
{
    const bool normal = fsm_.ctrl == E_NORMAL;
    const bool active = normal || fsm_.ctrl == E_STAND;
    float j1 = 0, j2 = 0, j3 = 0, j4 = 0;
    if (active) {
        j1 = output_.right_leg.len_ctrl_small + output_.right_leg.rad_ctrl_small
            + output_.right_leg.forward_cod_small + output_.right_leg.forward_poly_small;
        j2 = output_.right_leg.len_ctrl_big + output_.right_leg.rad_ctrl_big
            + output_.right_leg.forward_cod_big + output_.right_leg.forward_poly_big;
        j3 = output_.left_leg.len_ctrl_big + output_.left_leg.rad_ctrl_big
            + output_.left_leg.forward_cod_big + output_.left_leg.forward_poly_big;
        j4 = output_.left_leg.len_ctrl_small + output_.left_leg.rad_ctrl_small
            + output_.left_leg.forward_cod_small + output_.left_leg.forward_poly_small;
        if (normal && lqr_ctrl_) {
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
    motors_.right_forward->control(0, 0, 0, 0, j1 * J1_dir);
    motors_.right_back->control(0, 0, 0, 0, j2 * J2_dir);
    motors_.left_back->control(0, 0, 0, 0, j3 * J3_dir);
    motors_.left_forward->control(0, 0, 0, 0, j4 * J4_dir);
    // 轮轴 N·m 转原始电流指令，轮端只在 NORMAL 输出。
    constexpr float rotor_kt = WHEEL_ORIGINAL_TORQUE_CONSTANT / WHEEL_ORIGINAL_REDUCTION;
    constexpr float torque_to_current = WHEEL_MAX_RAW_CURRENT / (WHEEL_MAX_CURRENT * rotor_kt * WHEEL_REDUCTION);
    motors_.left->update(normal ? output_.lqr.T_w_l * LEFT_MOTOR_DIR * torque_to_current : 0.0f);
    motors_.right->update(normal ? output_.lqr.T_w_r * RIGHT_MOTOR_DIR * torque_to_current : 0.0f);
    if (restart_pending_) {
        restart();
        restart_pending_ = false;
    }

    if (active) {
        const auto& c = chassis_.get_data();
        const auto& b = base_.get_data();
        constexpr float sigma_Q = 0.2f, sigma_R_vel = 0.2f, sigma_R_acc = 0.6f;
        const float body_vel = kalman_vel_observer_.update(c.left_leg_state, c.right_leg_state, b,
                                                          sigma_Q, sigma_R_vel, sigma_R_acc);
        const float kalman_vel = kalman_vel_observer_.get_kalman_output(false);
        bsp_uart_printf(E_UART_1,"%f,%f,%f,%f,%f,%f\r\n",
                        (float)b.j1.state, (float)b.j2.state, (float)b.j3.state,
                        c.lqr_state.dot_x, body_vel, kalman_vel);
    }
}
