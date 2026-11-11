//
// Created by KLnormal on 2026/11/3.
//

#ifndef TROBOT_BASE_DATA_OBSERVER_H
#define TROBOT_BASE_DATA_OBSERVER_H
#include "ins/ins.h"
#include "motor/dji.h"
#include "motor/dm.h"

namespace Observer {
struct comp_ptr {
    motor::dji *left, *right;
    motor::dm *left_forward, *left_back, *right_forward, *right_back;
    const ins::data_t *ins_data_ptr;
};

struct joints {
    float pos_, vel_, tor; // rad [-π, π)、rad/s、N·m
    uint8_t state, master_id, slave_id; // state 是 DM 错误码
};

struct motors {
    float angle; // 轮轴单圈角，rad，[0, 2π)
    float total_angle; // 相对计零的有符号轮轴累计角，rad
    float angular_velocity, torque; // rad/s、N·m（电流估算）
    float distance, linear_velocity; // m、m/s，有符号
};

struct body_posture {
    float body_theta, body_phi, body_roll; // pitch、yaw、roll，rad
    float body_gyro_roll, body_gyro_pitch, body_gyro_yaw; // 绕机体 X/Y/Z 轴的角速度，rad/s
    float body_x_acc, body_y_acc, body_z_acc; // m/s²，未扣除重力
};

struct chassis_basedata {
    body_posture body;
    joints j1, j2, j3, j4; // 右前、右后、左后、左前
    motors motor_left, motor_right;
};

// 设备在对象存活期间有效；INS 已采用 X 前、Y 左、Z 上的机体坐标。
// 同一底盘任务按 WHEEL_CHASSIS_HZ 调用 tick、读取和重置，INS 任务优先级更高。
// timestamp=0 表示未收到反馈；相邻轮采样间转子转动必须严格小于半圈。
class Base_data_observer {
public:
    explicit Base_data_observer(comp_ptr comp):comp_(comp) {}
    void chassis_tick();
    chassis_basedata get_chassis_data() { return data_; }
    void reset_wheel_odometry() { reset_ = true; } // 下一次 tick 同时重置左右轮

private:
    struct wheel_history {
        int64_t rotor_count = 0;
        int16_t last_angle = 0;
        bool sampled = false;
    };
    static void wheel_process(const motor::dji::feedback_t &feedback, float dir,
                              wheel_history &history, motors &result);
    comp_ptr comp_;
    chassis_basedata data_ {};
    wheel_history left_, right_;
    bool reset_ = false;
};
}

#endif // TROBOT_BASE_DATA_OBSERVER_H
