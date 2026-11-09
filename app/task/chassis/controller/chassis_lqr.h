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
    }output;
    //LQR只负责LQR控制的模型计算部分内容，所以说请不要
    class ChassisLQR
    {
    public:
        ChassisLQR();
        ChassisLQR(bool fit); // true 使用腿长拟合，false 使用 static_matrix_K
        void lqr_clc(float target_vel, float target_omega,chassis_state state,
            float left_len, float right_len);
        void lqr_set_zero(chassis_state zero_state);
        void lqr_disable();
        void lqr_enable();
        output get_output() const { return output_; }
    private:
        void coef_clc(float left_len, float right_len);
        chassis_state zero_pos = {};
        chassis_state current_pos = {};
        chassis_state target_pos = {};
        float matrix_K[4][10] = {};
        output output_ = {};
        bool reference_ready = false;
        bool enable_flag = false;
        bool fit = false;
    };
}


#endif //TROBOT_CHASSIS_LQR_H
