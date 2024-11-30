/**
 * @file chassis.c
 * @author miNu50Ne (minun50ne@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "chassis_controller.h"

void ChassisInit()
{
    ChassisDeviceInit();
    ChassisParamInit();
    ChassisMsgInit();
}

void ChassisTask()
{
    ChassisModeSet();
    OmniCalculate();
    PowerController();
    ChassisMsgComm();
}