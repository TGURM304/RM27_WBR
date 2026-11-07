//
// Created by KLnormal on 2026/11/7.
//

#ifndef TROBOT_LEG_CONTROLLER_H
#define TROBOT_LEG_CONTROLLER_H

#include "controller/pid.h"
#include "leg_vmc.h"

namespace Controller
{
    typedef struct
    {
        //姿态PID控制和当前数据
        float rad_error; // rad，调用方计算的有符号误差
        float target_len;
        //当前状态
        float current_rad;
        float current_omega;
        float current_len;
        float current_speed;
        float small_rad, big_rad; // rad
        //两个前馈力
        float forward_x,forward_y;
        float forward_force_L, forward_tor_M;
    }leg_ctrl_data;
    typedef struct
    {
        //前馈的直角坐标系扭矩计算
        float forward_cod_small, forward_cod_big;
        //前馈的极坐标系扭矩计算
        float forward_poly_small, forward_poly_big;
        //腿长控制的极坐标系计算
        float pos_ctrl_small, pos_ctrl_big;
    }out_put_tor;
    class LegController {
        public:
        LegController();
        LegController(controller::pid::pid_para_t leg_len_parm, controller::pid::pid_para_t leg_speed_parm,
                controller::pid::pid_para_t leg_rad_parm, controller::pid::pid_para_t leg_omega_parm):
            len_(leg_len_parm),speed_(leg_speed_parm),rad_(leg_rad_parm),omega_(leg_omega_parm)
        {
            len_pid.init(len_);
            speed_pid.init(speed_);
            rad_pid.init(rad_);
            omega_pid.init(omega_);
        }
        void leg_ctrl_clc(leg_ctrl_data ctrl);
        void leg_clear();
        void leg_disable();
        void leg_enable();
        out_put_tor get_output(){return output_;};
        private:
            void leg_forward_clc();
            void leg_pos_ctrl();
            leg_ctrl_data leg_ctrl_{};
            out_put_tor output_{};
            bool ctrl_flag = false;
            controller::pid::pid_para_t len_, speed_, rad_, omega_;
            controller::pid len_pid, speed_pid, rad_pid, omega_pid;
    };
}



#endif //TROBOT_LEG_CONTROLLER_H
