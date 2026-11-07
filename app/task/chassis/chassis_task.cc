//
// Created by KLnormal on 2026/11/3.
//

#include "chassis_task.h"
#include "chassis_data_conifg.h"
#include "observer/base_data_observer.h"
#include "observer/leg_data_observer.h"
#include "bsp/uart.h"
#include "utils/os.h"
#include "leg_vmc.h"
#include "rc/dr16.h"
#include "controller/leg_controller.h"
#include <cmath>

namespace {
motor::dm::param_t joint_param(uint8_t id) {
    return {.slave_id = static_cast<uint8_t>(id + 0x10), .master_id = id,
            .port = E_CAN_2, .mode = motor::dm::MIT,
            .p_max = 3.141593f, .v_max = 45.0f, .t_max = 30.0f};
}
}

[[noreturn]] void chassis_task(void *) {
    motor::dm joint[] = {{"j1", joint_param(1)}, {"j2", joint_param(2)},
                         {"j3", joint_param(3)}, {"j4", joint_param(4)}};
    motor::dji right("right", motor::dji::M3508, {1, E_CAN_1, motor::dji::CURRENT});
    motor::dji left("left", motor::dji::M3508, {2, E_CAN_1, motor::dji::CURRENT});
    for(auto &motor : joint) {
        motor.init();
        motor.enable();
    }
    right.init();
    left.init();
    right.enable();
    left.enable();
    Observer::base_data_observer base({
        .left = &left, .right = &right,
        .left_forward = &joint[3], .left_back = &joint[2],
        .right_forward = &joint[0], .right_back = &joint[1], .ins_data_ptr = ins::data()
    });
    Observer::leg_observer legs(20.0f);
    Controller::LegController leg_ctrl(
    {10, 0, 0, 0, 0.8},
 {150, 10, 0, 20, 50},
    {1, 0, 0, 1, 1},
 {1, 0, 0, 1, 1});
    TickType_t wake_time = xTaskGetTickCount();

    auto rc = rc::dr16::data();
    bool enable_flag = false;
    Controller::out_put_tor out;
    Observer::chassis_basedata data = {};
    for(;;) {
        if (rc->s_l == 1)
            enable_flag = true;
        else if (rc->s_l != 1)
            enable_flag = false;
        if (enable_flag == false)
        {
            joint[0].control(0,0,0,0,0);
            joint[1].control(0,0,0,0,0);
            joint[2].control(0,0,0,0,0);
            joint[3].control(0,0,0,0,0);
        }
        else if (enable_flag == true)
        {
            joint[0].control(0,0,0,0,0);
            joint[1].control(0,0,0,0,0);
            joint[2].control(0,0,0,0,0);
            joint[3].control(0,0,0,0,0);
        }
        right.update(0);
        left.update(0);

        base.chassis_tick();
        data = base.get_chassis_data();
        // leg_tick 输入顺序：左小、左大、右小、右大，单位 rad。
        legs.leg_tick(data.j4.pos_, data.j3.pos_, data.j1.pos_, data.j2.pos_);
        const auto leg = legs.get_leg_data().first;
        Controller::leg_ctrl_data cmd{};
        cmd.rad_error = (PI/2 - leg.leg_phi);
        if (cmd.rad_error > PI) cmd.rad_error = 
        cmd.target_len = 0.0f;
        cmd.current_rad = leg.leg_phi; cmd.current_omega = leg.leg_phi_dot;
        cmd.current_len = leg.leg_len; cmd.current_speed = leg.leg_len_dot;
        cmd.small_rad = leg.joint_small; cmd.big_rad = leg.joint_big;
        cmd.forward_x = 0.0f; cmd.forward_tor_M = 0.0f;
        cmd.forward_y = 0.0f; cmd.forward_force_L = 0.0f;
        if (enable_flag) leg_ctrl.leg_enable(); else leg_ctrl.leg_disable();
        leg_ctrl.leg_ctrl_clc(cmd);
        out = leg_ctrl.get_output();
        bsp_uart_printf(E_UART_1,"%f,%f,%f,%f,%f,%f\r\n",
            out.forward_cod_small, out.forward_cod_big, out.forward_poly_small, out.forward_poly_big,
            out.pos_ctrl_small, out.pos_ctrl_big);
        vTaskDelayUntil(&wake_time, configTICK_RATE_HZ / WHEEL_CHASSIS_HZ);
    }
}
