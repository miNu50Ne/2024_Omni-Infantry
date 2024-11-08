#pragma once

#include "dji_motor.h"
#include "ramp.h"
#include <stdint.h>

#define RC_LOST (rc_data[TEMP].rc.switch_left == 0 && rc_data[TEMP].rc.switch_right == 0)

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#if PITCH_FEED_TYPE                                                  // Pitch电机反馈数据源为陀螺仪
#define PTICH_HORIZON_ANGLE 0                                        // PITCH水平时电机的角度
#if PITCH_ECD_UP_ADD
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#else
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif
#else                                                                   // PITCH电机反馈数据源为编码器
#define PTICH_HORIZON_ANGLE    (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // PITCH水平时电机的角度,0-360
#define PITCH_LIMIT_ANGLE_UP   (PITCH_POS_MAX_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (PITCH_POS_MIN_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif

#define MOUSEKEYCONTROL switch_is_up(rc_data[TEMP].rc.switch_left) && (switch_is_down(rc_data[TEMP].rc.switch_right))
#define ENTIREDISABLE   (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right))
// 底盘模式
#define CHASSIS_FREE     0
#define CHASSIS_ROTATION 1
#define CHASSIS_FOLLOW   2
#define SHOOT_FRICTION   3
#define SHOOT_LOAD       4

typedef struct {
    /*控制值*/
    uint8_t ui_refresh_flag; // UI发送标志位

    uint8_t rc_mode[5];

    uint8_t auto_aim;
    uint8_t auto_rune; // 自瞄打符标志位
    float rec_yaw, rec_pitch;

    float yaw_control;   // 遥控器YAW自由度输入值
    float pitch_control; // 遥控器PITCH自由度输入值
    float heat_coef;
    ramp_t *fb_ramp;
    ramp_t *lr_ramp;
} CmdInstance;

/* 初始化 */
/**
 * @brief cmd设备初始化，遥控器，裁判系统
 *
 */
void CmdDeviceInit();

/**
 * @brief cmd中介变量初始化
 *
 */
void CmdParamInit();

/**
 * @brief cmd消息收发初始化
 *
 */
void CmdMsgInit();

/*cmd任务*/
/**
 * @brief  裁判系统判断各种ID，选择客户端ID
 * @retval none
 * @attention
 */
void DeterminRobotID();

/**
 * @brief 自瞄手瞄切换
 *
 */
void GimbalModeSwitch();

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
void CalcOffsetAngle();

/**
 * @brief 对Pitch轴角度变化进行限位
 *
 */
void PitchAngleLimit();

/**
 * @brief 云台Yaw轴反馈值改单圈角度后过圈处理
 *
 */
void YawControlProcess();

/**
 * @brief 发射启动、热量控制
 *
 */
void ShootHeatControl();

/**
 * @brief cmd模式切换
 *
 */
void CmdModeSet();

/**
 * @brief cmd消息收发
 *
 */
void CmdMsgComm();
