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
        void chassis_move_clc(Coordinator::leg_state left_leg, Coordinator::leg_state right_leg, chassis_basedata chassis);
        [[nodiscard]] float get_acc_x() const { return acc_x; }
        // 纯滚动下的对地轮速估计，向前为正，单位 m/s。
        [[nodiscard]] float get_vel_x() const { return vel_wheel; }
    private:
        float acc_x = 0.0f; // 机体水平向前的加速度，m/s²。
        float vel_wheel = 0.0f; // 左右轮对地线速度的平均值。
        float vel_wheel_left = 0.0f, vel_wheel_right = 0.0f;
        bool enabled_ = false;
    };
}



#endif //TROBOT_CHASSIS_VEL_OBSERVER_H
