//
// Created by KLnormal on 2026/11/3.
//

#include "chassis_task.h"
#include "chassis_data_conifg.h"
#include "chassis_coor.h"
#include "bsp/uart.h"
#include "utils/os.h"

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
    Observer::comp_ptr motors{
        .left = &left, .right = &right,
        .left_forward = &joint[3], .left_back = &joint[2],
        .right_forward = &joint[0], .right_back = &joint[1], .ins_data_ptr = ins::data()
    };
    Observer::Base_data_observer base(motors);
    Observer::Leg_observer legs(20.0f);
    const controller::pid::pid_para_t len{5, 0, 0, 0, 0.8f}, speed{150, 10, 0, 20, 50};
    const controller::pid::pid_para_t rad{20, 0, 0, 0, 2}, omega{8, 1, 0, 3, 10};
    Controller::LegController leg_ctrl[] = {{len, speed, rad, omega}, {len, speed, rad, omega}};
    Controller::ChassisLQR lqr_ctrl_{};
    Coordinator::chassis_coor coor(&lqr_ctrl_, &leg_ctrl[0], &leg_ctrl[1], &base, &legs, &legs, motors);
    TickType_t wake_time = xTaskGetTickCount();
    for(;;) {
        coor.chassis_observer();
        coor.chassis_command();
        coor.chassis_protection();
        coor.chassis_scheduler();
        coor.chassis_controller();
        coor.chassis_actuator();
        vTaskDelayUntil(&wake_time, configTICK_RATE_HZ / WHEEL_CHASSIS_HZ);
    }
}
