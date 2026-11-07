//
// Created by KLnormal on 2026/11/5.
//

#include "leg_data_observer.h"

#include <cmath>
#include <numbers>

#include "chassis_data_conifg.h"

using namespace  Observer;

void leg_observer::leg_tick(float left_small, float left_big,
            float right_small, float right_big)
{
    if (leg_clear_flag == true)
    {
        leg_data_left = {0};
        leg_data_right = {0};
        leg_clear_flag = false;
        left_len_filter.clear();
        right_len_filter.clear();
        left_phi_filter.clear();
        right_phi_filter.clear();
        leg_data_left.old_phi = leg_data_left.leg_phi;
        leg_data_right.old_phi = leg_data_right.leg_phi;
        leg_data_left.old_len = leg_data_left.leg_len;
        leg_data_right.old_len = leg_data_right.leg_len;
    }

    leg_data_left.old_phi = leg_data_left.leg_phi;
    leg_data_right.old_phi = leg_data_right.leg_phi;
    leg_data_left.old_len = leg_data_left.leg_len;
    leg_data_right.old_len = leg_data_right.leg_len;

    leg_data_left.joint_small = left_small;
    leg_data_left.joint_big = left_big;
    leg_data_right.joint_small = right_small;
    leg_data_right.joint_big = right_big;

    leg_pos_clc(leg_data_left);
    leg_pos_clc(leg_data_right);
    leg_after_clc();
}

void leg_observer::leg_pos_clc(leg_data& data)
{
    const float delta = data.joint_big - data.joint_small;
    // 先取有向最短角再求半角，避免关节跨 +/-pi 时切换几何分支。
    const float half_angle = 0.5f * atan2f(sinf(delta), cosf(delta));
    const float beta = asinf(sinf(half_angle)*LEG_LEN_AD/LEG_LEN_CD);
    const float cd_angle = data.joint_big - half_angle - beta;
    data.leg_pos_x = cosf(data.joint_big)*LEG_LEN_AG + cosf(cd_angle)*LEG_LEN_GW;
    data.leg_pos_y = sinf(data.joint_big)*LEG_LEN_AG + sinf(cd_angle)*LEG_LEN_GW;
    data.leg_phi = atan2f(data.leg_pos_y, data.leg_pos_x);
    data.leg_len = sqrtf(data.leg_pos_x*data.leg_pos_x + data.leg_pos_y*data.leg_pos_y);
}

void leg_observer::leg_after_clc()
{
    leg_data *ptr[2]={&leg_data_left, &leg_data_right};
    for (auto i:ptr)
    {
        float delta_time = 1.0f/WHEEL_CHASSIS_HZ;
        float delta_phi = atan2f(sinf(i->leg_phi - i->old_phi),
                                 cosf(i->leg_phi - i->old_phi));
        float delta_len = i->leg_len - i->old_len;
        if (i == &leg_data_left)
        {
            left_raw = delta_len/delta_time;
            left_phi_raw = delta_phi/delta_time;
            i->leg_len_dot = left_len_filter.update(delta_len/delta_time);
            i->leg_phi_dot = left_phi_filter.update(delta_phi/delta_time);
        }
        else if (i == &leg_data_right)
        {
            i->leg_len_dot = right_len_filter.update(delta_len/delta_time);
            i->leg_phi_dot = right_phi_filter.update(delta_phi/delta_time);
        }
    }
}
