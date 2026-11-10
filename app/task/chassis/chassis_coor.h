//
// Created by KLnormal on 2026/11/9.
//

#ifndef TROBOT_CHASSIS_COOR_H
#define TROBOT_CHASSIS_COOR_H

//协调各个内容工作

#include "chassis_lqr.h"
#include "leg_controller.h"
#include "Base_data_observer.h"
#include "leg_data_observer.h"
#include "rc/dr16.h"
using namespace Controller;
using namespace Observer;


namespace Coordinator
{
    typedef struct {
        float current_rad;
        float current_omega;
        float current_len;
        float current_speed;
    }leg_state;
    typedef struct
    {
        chassis_state lqr_state;
        leg_state left_leg_state, right_leg_state;
        float kalman_acc, kalman_vel;
    }processed_data;
    class chassis_coor
    {
    public:
        chassis_coor() = delete;
        // lqr_ctrl 为 nullptr 时只控制双腿，不计算或输出 LQR。
        chassis_coor(ChassisLQR *lqr_ctrl, LegController *left_leg_ctrl,
            LegController *right_leg_ctrl, Base_data_observer *base_data_observer,
            Leg_observer *left_leg_observer, Leg_observer *right_leg_observer, comp_ptr motors)
        :lqr_ctrl_(lqr_ctrl), left_leg_ctrl_(left_leg_ctrl), right_leg_ctrl_(right_leg_ctrl),
        left_leg_observer_(left_leg_observer), right_leg_observer_(right_leg_observer),
        base_data_observer_(base_data_observer), motors_(motors)
        {
            remote = rc::dr16::data();

        };
        void chassis_observer();//调用下面的两个私有的结构体更新内容
        void chassis_command();//负责解析命令
        void chassis_protection();//作保护
        void chassis_scheduler();//做规划，对解析的命令转化成控制要求
        void chassis_controller();//控制机器人运动，计算机器人运动电机所需力矩
        void chassis_actuator();//输出电机力矩
    private:
        ChassisLQR *lqr_ctrl_;
        LegController *left_leg_ctrl_, *right_leg_ctrl_;
        Leg_observer *left_leg_observer_, *right_leg_observer_;
        Base_data_observer *base_data_observer_;
        comp_ptr motors_;
        bool static_leg = false, lqr_enabled_ = false;
        chassis_state lqr_state_{};
        leg_ctrl_data left_cmd_{}, right_cmd_{};
        output lqr_output_{};
        out_put_tor left_output_{}, right_output_{};

        //受到严格保护的更新和存储函数
    private:
        //以下内容需要为订阅发布制度的结构体，想要读写数据都需要通过下面的结构体，保证数据不会被乱写乱用
        //以下构型沿用了前文已经确定的基础内容并且流出了拓展的空间，想要新加东西可以直接修改下面两个结构体
        struct base_data
        {
        public:
            [[nodiscard]] const chassis_basedata& get_data() const{return base_data_;}
            void update_data(const chassis_basedata& value) { base_data_ = value; }
        private:
            chassis_basedata base_data_ = {};
        };//基础的经过简单加工后的电机数据
        struct chassis_data
        {
        public:
            [[nodiscard]] const processed_data& get_data() const{return processed_data_;}
            void update_data(const processed_data& value) { processed_data_ = value; }
        private:
            processed_data processed_data_ = {};
        };//更新初步加工后的观测数据
        base_data base_;
        chassis_data chassis_;
        const rc::dr16::data_t *remote;
    };
}


#endif //TROBOT_CHASSIS_COOR_H
