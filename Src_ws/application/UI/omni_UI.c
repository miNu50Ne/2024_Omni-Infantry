/**
 * @file omni_UI.c
 * @author miNu50Ne (minu50ne@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "UI_controller.h"

void UIInit()
{
    // while (UIDeviceInit());
    UIDeviceInit();
    UIMsgInit();
}

void UITask()
{
    UIRefresh();
    UIMsgComm();
}