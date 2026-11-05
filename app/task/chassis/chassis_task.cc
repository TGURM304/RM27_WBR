//
// Created by KLnormal on 2026/11/3.
//

#include "chassis_task.h"
#include "chassis_data_conifg.h"
#include "observer/base_data_observer.h"
#include "observer/leg_data_observer.h"
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
    Observer::base_data_observer base({
        .left = &left, .right = &right,
        .left_forward = &joint[3], .left_back = &joint[2],
        .right_forward = &joint[0], .right_back = &joint[1], .ins_data_ptr = ins::data()
    });
    Observer::leg_observer legs(20.0f);
    TickType_t wake_time = xTaskGetTickCount();
    for(;;) {
        for(auto &motor : joint) motor.control(0, 0, 0, 0, 0);
        right.update(0);
        left.update(0);

        base.chassis_tick();
            const auto data = base.get_chassis_data();
            // leg_tick 输入顺序：左小、左大、右小、右大，单位 rad。
            legs.leg_tick(data.j4.pos_, data.j3.pos_, data.j1.pos_, data.j2.pos_);
            const auto [left_leg, right_leg] = legs.get_leg_data();
            bsp_uart_printf_async(E_UART_1, "%f,%f,%f,%f,%f,%f\r\n",
                                  left_leg.leg_len, right_leg.leg_len, left_leg.leg_len_dot, right_leg.leg_len_dot, 0.0, 0.0);
        vTaskDelayUntil(&wake_time, configTICK_RATE_HZ / WHEEL_CHASSIS_HZ);
    }
}
