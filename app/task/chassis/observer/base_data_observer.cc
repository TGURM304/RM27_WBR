//
// Created by KLnormal on 2026/11/3.
//

#include "base_data_observer.h"
#include "chassis_data_conifg.h"
#include "bsp/sys.h"
#include <cmath>
#include <numbers>

namespace {
constexpr float pi = std::numbers::pi_v<float>;
constexpr float two_pi = 2.0f * pi;
constexpr double two_pi_double = 2.0 * std::numbers::pi;
// 沿用现有配置名称，含义为转子转数 / 轮轴转数；当前 0.3 为占位。
constexpr float reduction = WHEEL_REDUCTION;
// 按 dji.cc 的系数约定，将原装减速比下的 0.3 N·m/A 折算到转子侧。
constexpr float rotor_torque_constant = 0.3f / (3591.0f / 187.0f);
static_assert(reduction > 0 && WHEEL_RADIUS > 0 && WHEEL_CHASSIS_HZ > 0);

float joint_angle(float angle) {
    angle = std::fmod(angle, two_pi);
    if(angle >= pi) angle -= two_pi;
    if(angle < -pi) angle += two_pi;
    return angle;
}

Observer::joints joint_map(const motor::dm::feedback_t &feedback,
                         const motor::dm::param_t &param, float offset, float dir) {
    Observer::joints result {};
    result.master_id = param.master_id;
    result.slave_id = param.slave_id;
    if(feedback.timestamp == 0) return result;
    result.pos_ = joint_angle(dir * feedback.pos+ offset);
    result.vel_ = dir * feedback.vel;
    result.tor = dir * feedback.torque;
    result.state = feedback.err;
    return result;
}
}

void Observer::base_data_observer::chassis_tick() {
    motor::dji::feedback_t left, right;
    motor::dm::feedback_t joint[4];
    ins::data_t body;
    // 临界区只复制输入；保持 INS 任务优先级高于调用任务且整轮发布的约定。
    const auto state = bsp_sys_enter_critical();
    left = comp_.left->feedback;
    right = comp_.right->feedback;
    joint[0] = comp_.right_forward->feedback;
    joint[1] = comp_.right_back->feedback;
    joint[2] = comp_.left_back->feedback;
    joint[3] = comp_.left_forward->feedback;
    body = *comp_.ins_data_ptr;
    bsp_sys_exit_critical(state); // 与 enter 配对，恢复调度及中断屏蔽状态

    if(reset_) {
        left_ = {};
        right_ = {};
        data_.motor_left = {};
        data_.motor_right = {};
        reset_ = false;
    }
    wheel_process(left, LEFT_MOTOR_DIR, left_, data_.motor_left);
    wheel_process(right, RIGHT_MOTOR_DIR, right_, data_.motor_right);
    data_.j1 = joint_map(joint[0], *comp_.right_forward->get_param(), J1_OFFSET, J1_dir);
    data_.j2 = joint_map(joint[1], *comp_.right_back->get_param(), J2_OFFSET, J2_dir);
    data_.j3 = joint_map(joint[2], *comp_.left_back->get_param(), J3_OFFSET, J3_dir);
    data_.j4 = joint_map(joint[3], *comp_.left_forward->get_param(), J4_OFFSET, J4_dir);
    data_.body = {
        body.pitch, body.yaw, body.roll,
        body.gyro[0], body.gyro[1], body.gyro[2],
        body.accel[0], body.accel[1], body.accel[2]
    };
}

void Observer::base_data_observer::wheel_process(const motor::dji::feedback_t &feedback, float dir,
                                               wheel_history &history, motors &result) {
    if(feedback.timestamp == 0) return;
    if(history.sampled) {
        int delta = static_cast<int>(feedback.raw.angle) - history.last_angle;
        if(delta > 4096) delta -= 8192;
        if(delta < -4096) delta += 8192;
        history.count += delta;
    } else {
        history.sampled = true;
    }
    history.last_angle = feedback.raw.angle;
    const double total = dir * static_cast<double>(history.count) * two_pi_double / (8192.0 * reduction);
    double angle = std::fmod(total, two_pi_double);
    if(angle < 0) angle += two_pi_double;
    result.angle = static_cast<float>(angle);
    if(result.angle >= two_pi) result.angle = 0; // float 舍入时仍保持半开区间
    result.total_angle = static_cast<float>(total);
    result.angular_velocity = dir * feedback.raw.speed * two_pi / (60.0f * reduction);
    const float current = feedback.raw.current * (20.0f / 16384.0f);
    result.torque = dir * current * rotor_torque_constant * reduction;
    result.distance = static_cast<float>(total * WHEEL_RADIUS);
    result.linear_velocity = result.angular_velocity * WHEEL_RADIUS;
}
