/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "rm_referee.h"
#include <stdint.h>

#define VISION_USE_VCP // 使用虚拟串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */

// 云台参数
#define YAW_CHASSIS_ALIGN_ECD     7831 // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 1    // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD         5480 // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_POS_UP_LIMIT_ECD    4893 // 云台竖直方向高处限位编码器值,若对云台有机械改动需要修改
#define PITCH_POS_DOWN_LIMIT_ECD  5938 // 云台竖直方向低处限位编码器值,若对云台有机械改动需要修改

#define PITCH_FEED_TYPE           1 // 云台PITCH轴反馈值来源:编码器为0,陀螺仪为1
#define PITCH_INS_FEED_TYPE       1 // 云台PITCH轴陀螺仪反馈:角度值为0,弧度制为1
#define PITCH_ECD_UP_ADD          0 // 云台抬升时编码器变化趋势,增为1,减为0 (陀螺仪变化方向应相同)

// 发射参数
#define ONE_BULLET_DELTA_ANGLE 45    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 36.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE         8     // 拨盘一圈的装载量

// 底盘参数,单位为mm(毫米)
#define WHEEL_BASE             350  // 纵向轴距(前进后退方向)
#define TRACK_WIDTH            350  // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL           153  // 轮子半径
#define REDUCTION_RATIO_WHEEL  0.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

#define MOTOR_OUTPUT_DIRECTION 0 // 转子与输出同向为1，反向为0

#define HALF_WHEEL_BASE        (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH       (TRACK_WIDTH / 2.0f)    // 半轮距
#define PERIMETER_WHEEL        (RADIUS_WHEEL * 2 * PI) // 轮子周长

// 其他参数(尽量所有参数集中到此文件)
#define BUZZER_SILENCE           0 // 蜂鸣器静音,1为静音,0为正常

#define INS_YAW_ADDRESS_OFFSET   2 // 陀螺仪数据相较于云台的yaw的方向
#define INS_PITCH_ADDRESS_OFFSET 1 // 陀螺仪数据相较于云台的pitch的方向
#define INS_ROLL_ADDRESS_OFFSET  0 // 陀螺仪数据相较于云台的roll的方向

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */

// 底盘模式设置
typedef enum {
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

typedef enum {
    SUPER_USER_CLOSE = 0, // 用户关闭超电
    SUPER_USER_OPEN,
} cap_mode_e;

// 云台模式设置
typedef enum {
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum {
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum {
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum {
    LOAD_STOP = 0, // 停止发射
    LOAD_REVERSE,  // 反转
    LOAD_1_BULLET, // 单发
    LOAD_JAM,
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot/UI订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    uint16_t power_buffer;           // 60焦耳缓冲能量
    float chassis_power;             // 底盘瞬时功率
    uint8_t level;                   // 机器人等级
    uint16_t power_limit;            // 底盘功率限制
    uint8_t SuperCap_flag_from_user; // 超电的标志位

    float vx; // 前进方向控制量
    float vy; // 横移方向控制量
    float wz; // 旋转速度

    uint8_t reverse_flag; // 小陀螺反转

    // float chassis_cmd_velocity_vector; // 底盘速度控制矢量 单位:m/s

    float gimbal_error_angle;

    chassis_mode_e chassis_mode;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    friction_mode_e friction_mode;
    uint16_t shooter_heat_cooling_rate; // 枪口热量冷却
    uint16_t shooter_referee_heat;      // 17mm枪口热量
    uint16_t shooter_cooling_limit;     // 枪口热量上限
    float shoot_rate;                   // 连续发射的射频,unit per s,发/秒
    float bullet_speed;                 // 子弹速度
    float loader_rate;                  // 拨弹盘转速
} Shoot_Ctrl_Cmd_s;

// cmd发布的UI数据,由UI订阅
typedef struct
{

    referee_id_t robot_id_for_ui; // 机器人id
    uint8_t init_flag;            // 初始化完成标志位
    uint8_t ui_refresh_flag;      // UI发送标志位

    uint16_t chassis_attitude_angle; // 底盘姿态角
    uint16_t Cap_absorb_power_limit; // 超电吸收功率
    chassis_mode_e chassis_mode;     // 底盘模式
    uint8_t SuperCap_mode;           // 超电开关指示
    float SuperCap_voltage;          // 超电电压

    uint8_t rune_mode;

    friction_mode_e friction_mode; // 摩擦轮模式
} UI_Cmd_s;

// cmd发布的master数据，由master订阅
typedef struct {
    uint8_t frame_head[4];
    float ins_quat[4];
    uint8_t robot_id;
    uint8_t rune_mode;
} Master_Cmd_s;

/* ----------------gimbal/shoot/chassis/UI发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
    // 后续增加底盘的真实速度
    float real_vx;
    float real_vy;
    float real_wz;

    uint8_t CapFlag_open_from_real;
    float cap_voltage;
    uint16_t cap_get_power_limit;

} Chassis_Upload_Data_s;

typedef struct
{
    INS_Instance *gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
    uint16_t yaw_ecd;
} Gimbal_Upload_Data_s;

typedef struct
{
    int shooter_heat_control; // 热量控制
    float shooter_local_heat; // 本地热量
} Shoot_Upload_Data_s;

typedef struct
{
    // code to go here
} UI_Upload_Data_s;

typedef struct {
    float rec_yaw;
    float rec_pitch;
    uint8_t rec_flag;
} Master_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H