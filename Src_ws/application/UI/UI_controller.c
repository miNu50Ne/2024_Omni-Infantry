/**
 * @file omni_UI.c
 * @author MiNu50Ne (minu50ne@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-7-15
 *
 * @copyright Copyright (c) 2024
 *
 */

// app
#include "robot_def.h"
#include "UI_controller.h"

// module
#include "rm_referee.h"
#include "referee_protocol.h"
#include "referee_UI.h"

#include "rm_referee.h"
#include "message_center.h"

MyUIInit_Ptr MyuiInitLocal;
MyUIRefresh_Ptr MyUIRefreshLocal;
referee_info_t *referee_info_t_ptr;

static Publisher_t *ui_pub;
static Subscriber_t *ui_sub;
static UI_Cmd_s ui_cmd_recv;
static UI_Upload_Data_s ui_feedback_data;

/**
 * @brief UI静态UI绘制
 *
 */
static void UIStaticRefresh()
{
    // 清空UI
    UIDelete(&ui_cmd_recv.robot_id_for_ui, UI_Data_Del_ALL, 0);
    // 射击准心
    // 小陀螺
    // 打符(单发)
    // 摩擦轮
    // 电容
    // 发送
}

/**
 * @brief UI动态UI绘制
 *
 */
static void UIDynamicRefresh()
{
    // 小陀螺

    // 打符(单发)

    // 摩擦轮

    // 电容&电容电压

    // 动态UI发送
}

int8_t UIDeviceInit()
{
    if (ui_cmd_recv.init_flag != 1) {
        return -1; // 硬件未初始化
    }

    MyuiInitLocal    = UIStaticRefresh;
    MyUIRefreshLocal = UIDynamicRefresh;

    (*MyuiInitLocal)();

    return 0;
}

void UIMsgInit()
{
    ui_pub = PubRegister("ui_feed", sizeof(UI_Upload_Data_s));
    ui_sub = SubRegister("ui_cmd", sizeof(UI_Cmd_s));
}

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
void UIRefresh()
{
    if (!ui_cmd_recv.ui_refresh_flag) {
        (*MyuiInitLocal)();
        ui_cmd_recv.ui_refresh_flag = 1;
    } else {
        (*MyUIRefreshLocal)();
    }
}

void UIMsgComm()
{
    SubGetMessage(ui_sub, (void *)&ui_cmd_recv);

    PubPushMessage(ui_pub, (void *)&ui_feedback_data);
}