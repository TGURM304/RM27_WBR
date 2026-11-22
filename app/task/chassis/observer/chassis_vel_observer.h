//
// Created by KLnormal on 2026/11/13.
//

#ifndef TROBOT_CHASSIS_VEL_OBSERVER_H
#define TROBOT_CHASSIS_VEL_OBSERVER_H

#include "Base_data_observer.h"
#include "leg_data_observer.h"

namespace Observer
{
    struct velocity_diagnostics {
        float left_velocity, right_velocity; // m/s，已扣除腿运动，尚未扣除横向安装点转动
        float body_velocity, acc_x; // 机体前向速度测量及加速度，m/s、m/s²
        float yaw_rate; // 横向安装点转动对前向轮差的等效角速度，rad/s
        float predicted_velocity; // 本次校正前预测，m/s
        bool valid; // 已有预测，且本轮测量和 KF 更新有效
    };

    class ChassisVelObserver
    {
    public:
        ChassisVelObserver() = default;
        void enable();
        void disable();
        void clear();
        // 直接解算机体前向速度（m/s），补偿腿部运动，不更新 KF。
        float chassis_move_clc(Base_data_observer *base_observer, Leg_observer *left_observer,
                               Leg_observer *right_observer);
        // 返回本轮未滤波的机体前向速度（m/s），供串口比较。
        // 指针必须有效；调用方先更新基础及腿部观察器，本函数只读取已有结果。
        // Q/R 直接取配置中的协方差；slip_confidence 为 [0,1]。
        // 超过配置门限且 |gyro yaw|<3 rad/s 时使用打滑参数，切换保留滤波历史。
        float update(Base_data_observer *base_observer, Leg_observer *left_observer,
                     Leg_observer *right_observer, float slip_confidence = 0.0f);
        // 水平前向速度，m/s。默认加上轮轴相对机体的运动分量。
        [[nodiscard]] float get_kalman_output(bool include_wheel_motion = true) const {
            return include_wheel_motion ? kalman_vel + wheel_relative_vel_ : kalman_vel;
        }
        [[nodiscard]] velocity_diagnostics get_diagnostics() const { return diagnostics_; }
    private:
        // q_* 为每拍过程噪声方差，r_* 为测量噪声方差，Q/R 交叉项为零。
        bool chassis_kalman(float acc_x, float body_vel, double q_vv, double q_aa, double r_vv, double r_aa);
        float kalman_vel = 0.0f, kalman_acc = 0.0f;
        float wheel_relative_vel_ = 0.0f;
        double p_vv_ = 0.0, p_va_ = 0.0, p_aa_ = 0.0;
        bool enabled_ = false;
        bool kalman_initialized_ = false;
        velocity_diagnostics diagnostics_{};
    };
}



#endif //TROBOT_CHASSIS_VEL_OBSERVER_H
