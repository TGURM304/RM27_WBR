//
// Created by KLnormal on 2026/11/3.
//

#ifndef TROBOT_CHASSIS_DATA_CONIFG_H
#define TROBOT_CHASSIS_DATA_CONIFG_H

//所有内容均使用国际单位制

#define PI 3.14159f
#define PI_F32 3.14159f
//offset应该写当前坐标系下设置电机零点后的摆杆相对建模零点的角度
#define J1_OFFSET ((180.0-66.29)/180.0f*PI)
#define J2_OFFSET ((180.0+27.82)/180.0f*PI)
#define J3_OFFSET ((180.0+27.82)/180.0f*PI)
#define J4_OFFSET ((180.0-66.29)/180.0f*PI)
#define J1_dir (-1)
#define J2_dir (-1)
#define J3_dir (1)
#define J4_dir (1)

#define LEFT_MOTOR_DIR (-1)
#define RIGHT_MOTOR_DIR (1)

//机体的物理数据
#define WHEEL_RADIUS 0.058f
#define WHEEL_REDUCTION (268.0f/17.0f)
#define WHEEL_ORIGINAL_REDUCTION (3591.0f / 187.0f)
//驱动轮半轮距
#define WHEEL_DIS (0.4386f/2.0f)


// 原装减速器输出轴力矩常数，N·m/A。
#define WHEEL_ORIGINAL_TORQUE_CONSTANT 0.3f
#define WHEEL_MAX_CURRENT 20.0f
#define WHEEL_MAX_RAW_CURRENT 16384.0f

#define WHEEL_CHASSIS_HZ (1000)
#define CAR_WEIGHT 20.0f

#define LEG_LEN_AB 0.0945f
#define LEG_LEN_AD LEG_LEN_AB
#define LEG_LEN_BC 0.1125f
#define LEG_LEN_CD LEG_LEN_BC
#define LEG_LEN_DG 0.1155f
#define LEG_LEN_AG (LEG_LEN_AD+LEG_LEN_DG)
#define LEG_LEN_GW 0.250f

//稳定后退的行程
#define LEG_STATIC_OFFSET (0)
#define LEG_LEN_MAX (0.32f)
#define LEG_LEN_MIN (0.15f)

//机器人的保护和状态机相关宏定义
#define BODY_PITCH_SAFE (45.0f/180.0f*PI)
#define BODY_LEG_DEG_SAFE (45.0f/180.0f*PI)
#define STAND_LEG_LEN (0.14f)
#define NORMAL_INIT_HEIGHT (0.18f)
#define STAND_LEG_RAD (PI / 2.0f)
#define STAND_LEN_MIN (0.125f)
#define STAND_LEN_MAX (0.145f)
#define STAND_LEG_RAD_TOL (15.0f/180.0f*PI)

#define LQR_ANGLE_GUARD_DELAY_MS (500u)
#define LQR_KALMAN_VEL_DELAY_MS (3000u) // E_NORMAL 进入后，允许切换 KF 速度的等待时间
#define NORMAL_HEIGHT_MAX (0.33f)
#define NORMAL_HEIGHT_COMP_ANGLE (30.0f/180.0f*PI)
#define LEG_SUPPORT_FADE_START (30.0f/180.0f*PI)
#define LEG_SUPPORT_FADE_END (40.0f/180.0f*PI)

#define SLIP_GATE (0.5)

// 打滑检测初值，待实机标定；速度/增量单位 m/s，角速度单位 rad/s。
#define SLIP_GATE_VEL_ENTER (0.6f)
#define SLIP_GATE_VEL_RECOVER (0.4f) // 左右锁存标志解除的速度失配上限，m/s
#define SLIP_GATE_YAW_ENTER (0.3f)
#define SLIP_GATE_YAW_RECOVER (0.2f) // 左右锁存标志解除的偏航失配上限，rad/s
#define SLIP_GATE_DELTA_ENTER (10.0f)
#define SLIP_GATE_DELTA_RECOVER (5.0f) // 本拍 C 诊断强度的零点，m/s，不参与整车保持计数
#define SLIP_MAX_YAW_RATE (3.0f)
#define SLIP_MAX_BODY_ANGLE (15.0f / 180.0f * PI)
#define SLIP_AVERAGE_SAMPLES (4u)
#define SLIP_LEFT_ACCEL_CUTOFF_HZ (5.0f)
#define SLIP_LEFT_ACCEL_AVERAGE_SAMPLES (5u)
// 固定 1 kHz；置信度衰减系数 k=2/T²，由下面的衰减时长自动推导。
#define SLIP_NORM_RISE_STEP (0.1f)
#define SLIP_NORM_ONE_CONDITION_SCALE (1.0f / 4.0f)
#define SLIP_NORM_TWO_CONDITION_SCALE (3.0f / 5.0f)
#define SLIP_NORM_HOLD_TICKS (50u) // 两侧均未命中后保持置信度的拍数，1拍=1ms
#define SLIP_NORM_DECAY_TICKS (50u) // 开始衰减后，从满值1归零的拍数；较小初值更早归零
#define SLIP_NORM_DECAY_SECONDS (SLIP_NORM_DECAY_TICKS * 0.001)
#define SLIP_RECOVER_MS (200u) // 左右锁存标志的恢复确认时间，与整车置信度保持期独立
#define SLIP_WARMUP_MS (200u)
#define SLIP_FEEDBACK_TIMEOUT_MS (10u)
#define SLIP_MAX_INTERVAL_MS (5u)

// HERO_LEG 每拍直接使用的 Q/R 对角协方差，非标准差，不再乘 dt 换算。
#define SLIP_KF_CONFIDENCE_GATE (0.5f)
#define CHASSIS_KF_Q_VEL (0.2f)
#define CHASSIS_KF_Q_ACC (100.0f)
#define CHASSIS_KF_R_VEL (3.0f)
#define CHASSIS_KF_R_ACC (1.0f)
#define SLIP_KF_Q_VEL (CHASSIS_KF_Q_VEL/1000)
#define SLIP_KF_Q_ACC (CHASSIS_KF_Q_ACC/1000)
#define SLIP_KF_R_VEL (CHASSIS_KF_R_VEL*10000)
#define SLIP_KF_R_ACC (CHASSIS_KF_R_ACC*1000)

namespace chassis_config {
// 传感器到机体，绕 Z 轴旋转 180°。
inline constexpr float imu_sensor_to_body[9] = {
    -1.0f,  0.0f, 0.0f,
     0.0f, -1.0f, 0.0f,
     0.0f,  0.0f, 1.0f
};
}

#endif //TROBOT_CHASSIS_DATA_CONIFG_H
