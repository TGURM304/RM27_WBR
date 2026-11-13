//
// Created by KLnormal on 2026/11/7.
//

#ifndef TROBOT_CHASSIS_LQR_H
#define TROBOT_CHASSIS_LQR_H

#include "chassis_lqr_data.h"
//X=[s,ds,phi,dphi,theta_l_l,dtheta_l_l,theta_l_r,dtheta_l_r,theta_b,dtheta_b];
//u=[T_w_l,T_w_r,T_b_l,T_b_r]
//u=-kx
namespace Controller
{
    typedef struct
    {
        float x,theta,left_theta,right_theta,phi;
        float dot_x,dot_theta,dot_left_theta,dot_right_theta,dot_phi;
    }chassis_state;
    typedef struct
    {
        float T_w_l,T_w_r;
        float T_b_l,T_b_r;
        // 拟合模式的静态配平前馈分量，已计入对应的 T_*。
        float forward_T_wl,forward_T_wr;
        float forward_T_bl,forward_T_br;
    }output;
    // 根据底盘状态和速度指令计算左右轮及左右腿的模型扭矩。
    class ChassisLQR
    {
    public:
        // 默认禁用，控制状态和扭矩输出初始化为零。
        ChassisLQR() = default;

        // 三个计算入口每个周期择一调用。target_vel 单位为 m/s，target_omega 单位为 rad/s。。
        // gain 指向 4×10 矩阵，是固定的控制增益矩阵K。
        void lqr_clc_staticK(float target_vel, float target_omega, const chassis_state& state,
            const float (*gain)[10]);
        // 使用同样的固定 K 矩阵布局，小板凳平衡模式。
        void lqr_clc_bench(float target_vel, float target_omega, chassis_state state,
            const float (*gain)[10]);
        // 用当前左右腿长（m）拟合 K 和静态前馈 Ud，输出 K 反馈与 Ud 之和。
        void lqr_clc_fitted(float target_vel, float target_omega, const chassis_state& state,
            float left_len, float right_len);
        // 用给定状态重设位移和偏航零点，清空输出，下一次使能计算时重新对齐参考。
        void lqr_set_zero(chassis_state zero_state);
        // 停止控制，立即清空扭矩输出和积分参考。
        void lqr_disable();
        // 从禁用切换到使能时，以当前状态设零。
        void lqr_enable(const chassis_state& state);
        // 按值读取四路模型扭矩及各自的前馈分量，单位为 N·m。
        [[nodiscard]] output get_output() const { return output_; }
    private:
        void lqr_feedback_clc(float target_vel, float target_omega, const chassis_state& state,
            const float (*gain)[10], float feed_dis);
        void forward_feed(const float (&basis)[10]);
        void coef_clc(const float (&basis)[10]);
        chassis_state zero_pos = {};
        chassis_state current_pos = {};
        chassis_state target_pos = {};
        float matrix_K[4][10] = {};
        output output_ = {};
        bool reference_ready = false;
        bool enable_flag = false;
    };
}


#endif //TROBOT_CHASSIS_LQR_H
