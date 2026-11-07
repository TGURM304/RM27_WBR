//
// Created by KLnormal on 2026/11/5.
//

#ifndef TROBOT_LEG_DATA_OBSERVER_H
#define TROBOT_LEG_DATA_OBSERVER_H
#include <utility>

#include "chassis_data_conifg.h"
#include "filter/lowpass_filter.h"

namespace Observer
{
    typedef struct
    {
        float joint_small,joint_big;
        float leg_pos_x, leg_pos_y;
        float leg_phi, leg_len;//腿极坐标系
        float leg_phi_dot, leg_len_dot;
        float old_phi, old_len;
    }leg_data;
    class leg_observer
    {
    public:
        leg_observer():leg_observer(20.0f) {}
        leg_observer(float cut_frequency_):
        left_len_filter(cut_frequency_,WHEEL_CHASSIS_HZ),
            right_len_filter(cut_frequency_,WHEEL_CHASSIS_HZ),
            left_phi_filter(cut_frequency_,WHEEL_CHASSIS_HZ),
            right_phi_filter(cut_frequency_,WHEEL_CHASSIS_HZ)
        {

        };
        // 输入为 A->B、A->D 的绝对方向角（rad）；正常构型避开 A、B、D 共线死点。
        void leg_tick(float left_small, float left_big,
            float right_small, float right_big);
        void leg_clear() {leg_clear_flag = true;}
        std::pair<leg_data,leg_data> get_leg_data()
        {
            std::pair<leg_data,leg_data> result;
            result.first = leg_data_left;
            result.second = leg_data_right;
            return result;
        }
        float left_raw, left_phi_raw;
    private:
        bool leg_clear_flag = true; 
        leg_data leg_data_left = {0}, leg_data_right = {0};
        filter::low_pass_filter left_len_filter, right_len_filter;
        filter::low_pass_filter left_phi_filter, right_phi_filter;
        void leg_pos_clc(leg_data &data);
        void leg_after_clc();
    };
}


#endif //TROBOT_LEG_DATA_OBSERVER_H
