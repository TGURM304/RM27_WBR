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

#define WHEEL_RADIUS 0.058f
#define WHEEL_REDUCTION (268.0f/17.0f)
#define WHEEL_ORIGINAL_REDUCTION (3591.0f / 187.0f)
// 原装减速器输出轴力矩常数，N·m/A。
#define WHEEL_ORIGINAL_TORQUE_CONSTANT 0.3f
#define WHEEL_MAX_CURRENT 20.0f
#define WHEEL_MAX_RAW_CURRENT 16384.0f

#define WHEEL_CHASSIS_HZ (1000)


#define LEG_LEN_AB 0.0945f
#define LEG_LEN_AD LEG_LEN_AB
#define LEG_LEN_BC 0.1125f
#define LEG_LEN_CD LEG_LEN_BC
#define LEG_LEN_DG 0.1155f
#define LEG_LEN_AG (LEG_LEN_AD+LEG_LEN_DG)
#define LEG_LEN_GW 0.250f

#define LEG_STATIC_OFFSET (-0.30)

namespace chassis_config {
// 传感器到机体，绕 Z 轴旋转 180°。
inline constexpr float imu_sensor_to_body[9] = {
    -1.0f,  0.0f, 0.0f,
     0.0f, -1.0f, 0.0f,
     0.0f,  0.0f, 1.0f
};
}

#endif //TROBOT_CHASSIS_DATA_CONIFG_H
