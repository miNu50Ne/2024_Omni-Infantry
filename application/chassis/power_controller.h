#pragma once

#include "robot_def.h"
#include "super_cap.h"

/**
 * @brief supercap task,run in chassis task
 *
 * @param cap
 * @param chassis_recv_msg
 */
void PowerCtrlTask(SuperCapInstance *cap, Chassis_Ctrl_Cmd_s chassis_recv_msg);
