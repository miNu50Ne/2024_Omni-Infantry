/**
 * @file gimbal.c
 * @author miNu50Ne (minu50ne@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "gimbal_controller.h"

void GimbalInit()
{
    GimbalDeviceInit();
    GimbalMsgInit();
}

void GimbalTask()
{
    GimbalModeSet();
    GimbalMsgComm();
}