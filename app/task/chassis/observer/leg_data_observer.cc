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
    const bool reset_history = leg_clear_flag;
    if (leg_clear_flag == true)
    {
        leg_data_left = {0};
        leg_data_right = {0};
        leg_clear_flag = false;
        left_len_filter.clear();
        right_len_filter.clear();
        left_phi_filter.clear();
        right_phi_filter.clear();
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
    if(reset_history)
    {
        leg_data_left.old_phi = leg_data_left.leg_phi;
        leg_data_right.old_phi = leg_data_right.leg_phi;
        leg_data_left.old_len = leg_data_left.leg_len;
        leg_data_right.old_len = leg_data_right.leg_len;
    }
    leg_after_clc();
}

void leg_observer::leg_pos_clc(leg_data& data)
{
    const float bx = LEG_LEN_AB * cosf(data.joint_small);
    const float by = LEG_LEN_AB * sinf(data.joint_small);
    const float dx = LEG_LEN_AD * cosf(data.joint_big);
    const float dy = LEG_LEN_AD * sinf(data.joint_big);
    const float bd_x = dx - bx, bd_y = dy - by;
    const float bd_len = hypotf(bd_x, bd_y);
    if(bd_len == 0.0f) return; // B、D 重合时 C 不唯一，保留上次位置
    const float projection = (LEG_LEN_BC * LEG_LEN_BC - LEG_LEN_CD * LEG_LEN_CD
                            + bd_len * bd_len) / (2.0f * bd_len);
    // 两圆交点中，选择 C 与 A 位于 B-D 连线两侧的装配分支。
    const float side = bx * dy - by * dx > 0.0f ? -1.0f : 1.0f;
    const float height = side * sqrtf(LEG_LEN_BC * LEG_LEN_BC - projection * projection);
    const float cx = bx + (projection * bd_x - height * bd_y) / bd_len;
    const float cy = by + (projection * bd_y + height * bd_x) / bd_len;
    data.leg_pos_x = LEG_LEN_AG * cosf(data.joint_big) + LEG_LEN_GW * (cx - dx) / LEG_LEN_CD;
    data.leg_pos_y = LEG_LEN_AG * sinf(data.joint_big) + LEG_LEN_GW * (cy - dy) / LEG_LEN_CD;
    data.leg_phi = atan2f(data.leg_pos_y, data.leg_pos_x);
    data.leg_len = hypotf(data.leg_pos_x, data.leg_pos_y);
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
