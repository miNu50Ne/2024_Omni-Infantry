/*
 * @Author: HDC h2019dc@outlook.com
 * @Date: 2023-09-08 16:47:43
 * @LastEditors: HDC h2019dc@outlook.com
 * @LastEditTime: 2023-10-18 23:26:21
 * @FilePath: \2024_Control_New_Framework_Base-dev-all\modules\super_cap\super_cap.h
 * @Description:
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2023-10-18 23:01:07
 */
#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "bsp_can.h"

#pragma pack(1)

#define SUPER_USER_OPEN     1
#define SUPER_USER_CLOSE    0

#define SUPER_VOLTAGE_OPEN  1
#define SUPER_VOLTAGE_CLOSE 0

#define SUPER_CMD_OPEN      1
#define SUPER_CMD_CLOSE     0

// 定义电压阈值
#define SUPER_VOLTAGE_THRESHOLD_LOW  10.0f
#define SUPER_VOLTAGE_THRESHOLD_HIGH 18.0f

#define SUPERCAP_PMOS_OPEN           1
#define SUPERCAP_PMOS_CLOSE          0

/* 超级电容发送信息 */
typedef struct
{
    float CapVot;                         // 电压
    float chassis_power_from_cap;         // 底盘功率
    float chassis_voltage_from_cap;       // 底盘电压
    uint8_t SuperCap_open_flag_from_real; // 开关指示 未开启为1
} SuperCap_Msg_s;

/* 超级电容接收信息 */
typedef struct
{
    uint16_t unused[3];
    uint8_t power_limit; // 功率限制
    uint8_t enabled;     // 电容开启状态
} SuperCap_Msg_g;
#pragma pack()

/* 超级电容实例 */
typedef struct
{
    CANInstance *can_ins;     // CAN实例
    SuperCap_Msg_s cap_msg_s; // 超级电容发送,本机接受的信息
    SuperCap_Msg_g cap_msg_g; // 超级电容接收,本机发送的信息
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

// 状态机
typedef enum {
    SUPER_STATE_LOW = 0,
    SUPER_STATE_HIGH,
    SUPER_STATE_ACCEL,
    SUPER_STATE_DECELER,
} SuperCap_State_e;

/**
 * @brief 初始化超级电容
 * @attention:data是超级电容接收信息的数组，只有四位，注意不要超出范围
 * @param supercap_config 超级电容初始化配置
 * @return SuperCapInstance* 超级电容实例指针
 */
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config);

/**
 * @brief 发送超级电容控制信息
 *
 * @param instance 超级电容实例
 * @param data 超级电容控制信息
 */
void SuperCapSend(SuperCapInstance *instance, uint8_t *data);

#endif // !SUPER_CAP_Hd
