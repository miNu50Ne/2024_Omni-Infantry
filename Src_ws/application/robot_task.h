/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

#include "daemon.h"

#include "bsp_log.h"
#include "ins_task.h"

#include "robot_cmd.h"
#include "master.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "motor_task.h"
#include "omni_UI.h"

//! 任务直接在cubeMX中配置,不再使用这种方式
__attribute__((noreturn)) void StartINSTASK(void *argument)
{
    UNUSED(argument);
    static uint32_t ins_time;
    static float ins_dt;
    LOGINFO("[freeRTOS] INS Task Start");
    while (1) {
        INS_Task();
        ins_dt = 1000 * DWT_GetDeltaT(&ins_time);
        if (ins_dt > 1.2f)
            LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]ms", &ins_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void _RobotCMDTask(void *argument)
{
    static uint32_t cmd_time;
    static float cmd_dt;
    LOGINFO("[freeRTOS] RobotCMD Task Start");
    for (;;) {
        RobotCMDTask();
        cmd_dt = 1000 * DWT_GetDeltaT(&cmd_time);
        if (cmd_dt > 1.2f)
            LOGERROR("[freeRTOS] Robot Task is being DELAY! dt = [%f]ms", &cmd_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void _MasterTask(void *argument)
{
    for (;;) {
        MasterTask();
        osDelay(1);
    }
}

__attribute__((noreturn)) void _ChassisTask(void *argument)
{
    for (;;) {
        ChassisTask();
        osDelay(1);
    }
}

__attribute__((noreturn)) void _GimbalTask(void *argument)
{
    for (;;) {
        GimbalTask();
        osDelay(1);
    }
}
__attribute__((noreturn)) void _ShootTask(void *argument)
{
    for (;;) {
        ShootTask();
        osDelay(1);
    }
}
__attribute__((noreturn)) void motorControlTask(void *argument)
{
    for (;;) {
        MotorControlTask();
        osDelay(1);
    }
}

__attribute__((noreturn)) void _DaemonTask(void *argument)
{
    for (;;) {
        DaemonTask();
        osDelay(1);
    }
}
