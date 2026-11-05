//
// Created by KLnormal on 2026/11/3.
//

#include "chassis_task.h"
#include "chassis_data_conifg.h"
#include "observer/base_data_observer.h"
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
    right.enable();
    left.enable();
    right.init();
    left.init();
    right.update(0);
    left.update(0);
    Observer::base_data_observer observer({
        .left = &left, .right = &right,
        .left_forward = &joint[3], .left_back = &joint[2],
        .right_forward = &joint[0], .right_back = &joint[1], .ins_data_ptr = ins::data()
    });
    volatile bool reset_odometry = false; // 调试器置 true，下一轮重置左右轮里程
    TickType_t wake_time = xTaskGetTickCount();
    static_assert(configTICK_RATE_HZ % WHEEL_CHASSIS_HZ == 0 && WHEEL_CHASSIS_HZ >= 10);
    for(;;) {
        for(auto &motor : joint) motor.control(0, 0, 0, 0, 0); // 零目标力矩 MIT 帧，请求反馈
        if(reset_odometry) {
            observer.reset_wheel_odometry();
            reset_odometry = false;
        }
        observer.chassis_tick();
        right.update(0);
        left.update(0);
        auto temp = observer.get_chassis_data();
        bsp_uart_printf(E_UART_1,"%f,%f,%f,%f,%f,%f\r\n",temp.j1.pos_,temp.j2.pos_,temp.j3.pos_,temp.j4.pos_,temp.motor_left.total_angle,temp.motor_right.total_angle);
        vTaskDelayUntil(&wake_time, configTICK_RATE_HZ / WHEEL_CHASSIS_HZ);
    }
}
