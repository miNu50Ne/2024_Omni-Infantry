/**
 * @file robot_cmd.c
 * @author miNu50Ne (minu50ne@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "cmd_controller.h"

void RobotCMDInit()
{
    CmdDeviceInit();
    CmdParamInit();
    CmdMsgInit();
}

void RobotCMDTask()
{
    DeterminRobotID();
    GimbalModeSwitch();
    CalcOffsetAngle();
    PitchAngleLimit();
    YawControlProcess();
    ShootHeatControl();
    CmdModeSet();
    CmdMsgComm();
}