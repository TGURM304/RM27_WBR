//
// Created by KLnormal on 2026/11/6.
//

#ifndef TROBOT_LEG_VMC_H
#define TROBOT_LEG_VMC_H
#include <utility>


std::pair<float,float> vmc_pol_clc(float theta1, float theta2,
    float tor_M, float force_L);
std::pair<float,float> vmc_co_clc(float theta1, float theta2, float force_x,
    float force_y, float leg_phi);


#endif //TROBOT_LEG_VMC_H
