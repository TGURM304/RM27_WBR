//
// Created by KLnormal on 2026/11/9.
//

#ifndef TROBOT_CHASSIS_COOR_H
#define TROBOT_CHASSIS_COOR_H

//协调各个内容工作

#include "chassis_lqr.h"
#include "leg_controller.h"
#include "Base_data_observer.h"
#include "chassis_vel_observer.h"
#include "leg_data_observer.h"
#include "rc/dr16.h"
#include "filter/window_filter.h"
#include "filter/lowpass_filter.h"
using namespace Controller;
using namespace Observer;


namespace Coordinator
{
    struct leg_state {
        float current_rad;
        float current_omega;
        float current_len;
        float current_speed;
    };
    typedef struct
    {
        chassis_state lqr_state;
        leg_state left_leg_state, right_leg_state;
        float kalman_acc, kalman_vel;
    }processed_data;
    struct chassis_output
    {
        Controller::output lqr;
        out_put_tor left_leg, right_leg;
    };
    struct slip_data {
        bool valid, left_slip, right_slip, yaw_slip, inertial_reference;
        float left_velocity, right_velocity; // 还原到机体中心，m/s
        float left_residual, right_residual; // B：轮速幅值减 KF 预测速度幅值，m/s
        float yaw_residual; // 有符号偏航失配，rad/s；yaw_slip 只记录 A 超限
        float left_delta_residual, right_delta_residual; // C：滤波轮加速度幅值减滤波 IMU 加速度幅值，m/s²
        float left_norm, right_norm; // 本拍左右证据强度 [0,1]
        float slip_norm; // 统一累计的整车置信度 [0,1]
        float reference_velocity, dt; // KF 更新前预测（m/s）、实际间隔（s）；inertial_reference 恒为 false
    };
    typedef enum
    {
        E_REST,
        E_STAND,
        E_NORMAL,
        E_STEP,
        E_JUMP,
        E_ERROR
    }ctrl_enum;//用于枚举机器人当前状态
    typedef struct
    {
        ctrl_enum ctrl;//底盘的控制状态机
        //打滑证据强度 [0,1]，采用带饱和的分段线性映射；锁存状态另含滞回
        //门限和归一化处理的内容可以在chassis_data_config.h文件中的SLIP_GATE_xxx中进行调配
        float slip_norm;
    }chassis_fsm;
    class chassis_coor
    {
    public:
        chassis_coor() = delete;
        // lqr_ctrl 为 nullptr 时跳过 LQR 计算。
        chassis_coor(ChassisLQR *lqr_ctrl, LegController *left_leg_ctrl,
            LegController *right_leg_ctrl, Base_data_observer *base_data_observer,
            Leg_observer *left_leg_observer, Leg_observer *right_leg_observer, comp_ptr motors)
        :lqr_ctrl_(lqr_ctrl), left_leg_ctrl_(left_leg_ctrl), right_leg_ctrl_(right_leg_ctrl),
        left_leg_observer_(left_leg_observer), right_leg_observer_(right_leg_observer),
        base_data_observer_(base_data_observer), motors_(motors)
        {};
        void chassis_observer();//调用下面的两个私有的结构体更新内容
        void chassis_command();//负责解析命令
        void chassis_condition();//做状态机规划，修改当前的控制状态
        void chassis_scheduler();//做规划，对解析的命令转化成控制要求
        void chassis_controller();//控制机器人运动，计算机器人运动电机所需力矩
        void chassis_actuator();//输出电机力矩
        [[nodiscard]] const chassis_output& get_output() const { return output_; }
        [[nodiscard]] ctrl_enum get_state() const { return fsm_.ctrl; }
        [[nodiscard]] slip_data get_slip_data() const { return slip_; }
    //不同状态机的控制函数
    private:
        void getup();//收腿到0.14m并将腿调整到机体系竖直
        void base_chassis_ctrl();//基础的底盘控制，腿长+移动
        void jump();//跳跃功能
        void step_up();//磕腿上台阶
        void rest();//全部电机输出0，并且重新置零所有电机和观测器的积分累计内容
        void restart();//重新置位和重新reset所有电机，然后进行enable
        void slip_clc();//计算轮腿打滑的可信度
    //成员变量
    private:
        ChassisLQR *lqr_ctrl_;
        LegController *left_leg_ctrl_, *right_leg_ctrl_;
        Leg_observer *left_leg_observer_, *right_leg_observer_;
        Base_data_observer *base_data_observer_;
        comp_ptr motors_;
        bool state_entry_ = true, restart_pending_ = false;
        bool remote_seen_ = false, last_enabled_ = false, start_request_ = false;
        uint32_t normal_start_ms_ = 0;
        float target_vel_ = 0.0f, target_omega_ = 0.0f; // m/s、rad/s
        float target_leg_vel_ = 0.0f; // 高度变化速度，m/s
        float target_height_ = NORMAL_INIT_HEIGHT; // m
        chassis_state lqr_state_{};
        leg_ctrl_data left_cmd_{}, right_cmd_{};
        chassis_output output_{};
        chassis_fsm fsm_{};
        struct slip_history {
            uint64_t start_us = 0, last_us = 0;
            uint64_t recover_us[2]{};
            unsigned norm_quiet_ticks = 0, norm_decay_ticks = 0;
            float norm_decay_start = 0.0f;
            float last_left_wheel_velocity = 0.0f, last_right_velocity = 0.0f;
            filter::moving_average_filter<SLIP_LEFT_ACCEL_AVERAGE_SAMPLES> left_acc_average{};
            filter::low_pass_filter left_acc_filter{SLIP_LEFT_ACCEL_CUTOFF_HZ, WHEEL_CHASSIS_HZ};
            filter::moving_average_filter<SLIP_AVERAGE_SAMPLES> right_delta_filter{}, imu_delta_filter{};
            bool tracking = false;
        } slip_history_{};
        slip_data slip_{};
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
        ChassisVelObserver kalman_vel_observer_{};
        rc::dr16::data_t remote_{};
    };
}


#endif //TROBOT_CHASSIS_COOR_H
