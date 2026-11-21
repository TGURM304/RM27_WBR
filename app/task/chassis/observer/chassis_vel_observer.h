//
// Created by KLnormal on 2026/11/13.
//

#ifndef TROBOT_CHASSIS_VEL_OBSERVER_H
#define TROBOT_CHASSIS_VEL_OBSERVER_H

#include "Base_data_observer.h"
namespace Coordinator
{
    struct leg_state;
}

namespace Observer
{
    class ChassisVelObserver
    {
    public:
        ChassisVelObserver() = default;
        void enable();
        void disable();
        void clear();
        // 返回本轮未滤波的机体前向速度（m/s），供串口比较。
        // 标准差单位依次为 m/s³、m/s、m/s²。两路测量标准差均为 +∞ 时仅预测。
        float update(const Coordinator::leg_state& left_leg, const Coordinator::leg_state& right_leg,
                     const chassis_basedata& chassis, float sigma_Q, float sigma_R_vel, float sigma_R_acc);
        // 水平前向速度，m/s。默认加上轮轴相对机体的运动分量。
        [[nodiscard]] float get_kalman_output(bool include_wheel_motion = true) const {
            return include_wheel_motion ? kalman_vel + wheel_relative_vel_ : kalman_vel;
        }
    private:
        bool chassis_kalman(float body_vel, float acc_x, float sigma_Q, float sigma_R_vel, float sigma_R_acc);
        float kalman_vel = 0.0f, kalman_acc = 0.0f;
        float wheel_relative_vel_ = 0.0f;
        double p_vv_ = 0.0, p_va_ = 0.0, p_aa_ = 0.0;
        bool enabled_ = false;
        bool kalman_initialized_ = false;
    };
}



#endif //TROBOT_CHASSIS_VEL_OBSERVER_H
