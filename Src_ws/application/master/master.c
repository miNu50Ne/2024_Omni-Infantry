/**
 * @file master.c
 * @author miNu50Ne (minu50ne@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-06
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "master_controller.h"

void MasterInit()
{
    MasterDeviceInit();
    MasterMsgInit();
}

void MasterTask()
{
    MasterMsgProcess();
    MasterMsgComm();
}