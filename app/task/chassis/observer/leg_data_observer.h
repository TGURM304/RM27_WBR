//
// Created by KLnormal on 2026/11/5.
//

#ifndef TROBOT_LEG_DATA_OBSERVER_H
#define TROBOT_LEG_DATA_OBSERVER_H
#include <utility>

namespace Observer
{
    typedef struct
    {
        float joint_small,joint_big;
        float leg_pos_x, leg_pos_y;
        float leg_phi, leg_len;//腿极坐标系
        float left_phi_dot, leg_len_dot;
    }leg_data;
    class leg_observer
    {
    public:
        leg_observer();
        leg_observer(float low_pass);
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
    private:
        bool leg_clear_flag = false;
        float old_left_phi = 0, old_right_phi = 0;
        float old_left_len, old_right_len;
        leg_data leg_data_left = {0}, leg_data_right = {0};
        void leg_pos_clc(leg_data &data);
        void leg_after_clc(leg_data &data);
    };
}


#endif //TROBOT_LEG_DATA_OBSERVER_H
