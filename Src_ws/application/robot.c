/**
 * @Author: HDC h2019dc@outlook.com
 * @Date: 2023-09-08 16:47:43
 * @LastEditors: HDC h2019dc@outlook.com
 * @LastEditTime: 2023-10-26 21:51:44
 * @FilePath: \2024_Control_New_Framework_Base-dev-all\application\robot.c
 * @Description:
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
#include "bsp_init.h"
#include "buzzer.h"

#include "robot.h"
#include "robot_task.h"

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#include "master.h"
#include "omni_UI.h"

void RobotInit()
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();

    BSPInit();
    buzzer_one_note(Do_freq, 0.1f);
    RobotCMDInit();
    buzzer_one_note(Re_freq, 0.1f);
    GimbalInit();
    buzzer_one_note(Mi_freq, 0.1f);
    ShootInit();
    buzzer_one_note(Fa_freq, 0.1f);
    ChassisInit();
    buzzer_one_note(So_freq, 0.1f);
    MasterInit();
    buzzer_one_note(La_freq, 0.1f);
    UIInit();
    buzzer_one_note(Si_freq, 0.1f);
    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
    MasterTask();
    RobotCMDTask();
    GimbalTask();
    ShootTask();
    ChassisTask();
    UITask();
}