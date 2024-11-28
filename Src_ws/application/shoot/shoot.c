/**
 * @file shoot.c
 * @author miNu50Ne (minu50ne@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "shoot_controller.h"

void ShootInit()
{
    ShootDeviceInit();
    ShootParamInit();
    ShootMsgInit();
}

void ShootTask()
{
    ShootModeSet();
    // loader_status_update();
    ShootMsgComm();
}