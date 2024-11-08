#pragma once

#include <stdint.h>
#include "referee_protocol.h"

typedef void (*MyUIInit_Ptr)(void);
typedef void (*MyUIRefresh_Ptr)(void);

typedef struct {
    Graph_Data_t shoot_line[7];   // 射击准线
    Graph_Data_t state_circle[4]; // 圆形
    String_Data_t Char_State[6];  // 字符串

    Graph_Data_t Cap_voltage;      // 电容电压
    Graph_Data_t Shoot_Local_Heat; // 射击本地热量
} UI_Graph_t;

/**
 * @brief UI设备初始化，UI绘制函数（设备）
 *
 * @param MyuiInit
 * @param MyUIRefresh
 * @return int8_t
 */
int8_t UIDeviceInit();

/**
 * @brief UI消息收发初始化
 *
 */
void UIMsgInit();

/**
 * @brief UI刷新任务
 *
 */
void UIRefresh();

/**
 * @brief UI消息收发
 *
 */
void UIMsgComm();