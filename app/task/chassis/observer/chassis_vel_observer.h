//
// Created by KLnormal on 2026/11/13.
//

#ifndef TROBOT_CHASSIS_VEL_OBSERVER_H
#define TROBOT_CHASSIS_VEL_OBSERVER_H

#include "filter/window_filter.h"

namespace Observer
{
    typedef struct
    {
        float acc_x;//IMU X 轴最近8次平均加速度，m/s²；窗口未满使用已有样本
        float hip_vel_left, hip_vel_right;//左右髋关节的速度，slip中有
    }chassis_raw;
    typedef struct
    {
        chassis_raw raw_data;
        float kalman_vel, kalman_acc;
        double p_vv, p_va, p_aa; // 后验协方差，P 对称，仅保存三个独立元素
    }kalm_data;
    class ChassisVelObserver
    {
    public:
        ChassisVelObserver() = default;
        void enable();
        void disable();
        void clear();
        // 固定 1 ms；加速度 m/s²、髋部速度 m/s。Q/R 为每拍协方差，非标准差。
        // 输入须有限，q_* >= 0、r_* > 0；失能时不更新，首帧以观测初始化，P=I。
        void kalman_clc(float raw_acc, float raw_left, float raw_right,
         float q_vv, float q_aa, float r_vv, float r_aa);
        [[nodiscard]] kalm_data get_kalman_data() const {return data_;}
    private:
        bool enabled_ = false;
        bool initialized_ = false;
        kalm_data data_{};
        filter::moving_average_filter<8> acc_filter_{};
    };
}



#endif //TROBOT_CHASSIS_VEL_OBSERVER_H
