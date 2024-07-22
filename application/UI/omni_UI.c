// app
#include "robot_def.h"
#include "omni_UI.h"
// module
#include "rm_referee.h"
#include "referee_protocol.h"
#include "referee_UI.h"
#include "string.h"
#include "crc_ref.h"
#include "stdio.h"
#include "rm_referee.h"
#include "message_center.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "UI_interface.h"

static Publisher_t *ui_pub;
static Subscriber_t *ui_sub;
static UI_Cmd_s ui_cmd_recv;
static UI_Upload_Data_s ui_feedback_data;

referee_info_t *referee_data_for_ui;

uint8_t UI_rune; // 自瞄打符标志位

uint8_t UI_Seq;                            // 包序号，供整个referee文件使用
static UI_GRAPH_INSTANCE *shoot_line[4];   // 射击准线
static UI_GRAPH_INSTANCE *state_circle[4]; // 圆形
static UI_STRING_INSTANCE *Char_State[10]; // 字符串

static UI_GRAPH_INSTANCE *Cap_voltage;         // 电容电压
static UI_GRAPH_INSTANCE *Chassis_Ctrl_Power;  // 底盘控制功率（目标功率）
static UI_GRAPH_INSTANCE *Cap_Absorb_Power;    // 电容吸收功率
static UI_GRAPH_INSTANCE *Chassis_Power_Limit; // 底盘功率上限
static UI_GRAPH_INSTANCE *Shoot_Local_Heat;    // 射击本地热量
static UI_GRAPH_INSTANCE *Heat_Limit;          // 热量上限

static void UI_StaticInit()
{
    // 清空UI
    UIDelete(&referee_data_for_ui->referee_id, UI_Data_Del_ALL, 0);

    // 射击准心
    shoot_line[0] = UI_Graph_Init(GraphType_Line, 1, 9, UI_Color_White, 1, SCREEN_LENGTH / 2 - 20, SCREEN_WIDTH / 2 + 5, SCREEN_LENGTH / 2 - 20, SCREEN_WIDTH / 2 - 45);
    shoot_line[1] = UI_Graph_Init(GraphType_Line, 1, 9, UI_Color_White, 1, SCREEN_LENGTH / 2 - 70, SCREEN_WIDTH / 2 - 50, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 50);
    shoot_line[2] = UI_Graph_Init(GraphType_Line, 1, 9, UI_Color_White, 1, SCREEN_LENGTH / 2 - 20, SCREEN_WIDTH / 2 - 55, SCREEN_LENGTH / 2 - 20, SCREEN_WIDTH / 2 - 500);
    shoot_line[3] = UI_Graph_Init(GraphType_Line, 1, 9, UI_Color_White, 1, SCREEN_LENGTH / 2 - 15, SCREEN_WIDTH / 2 - 50, SCREEN_LENGTH / 2 + 30, SCREEN_WIDTH / 2 - 50);

    // 小陀螺
    Char_State[0]   = UI_String_Init(0, 7, UI_Color_Cyan, 23, 4, 580, 125, "Rotate");
    state_circle[0] = UI_Graph_Init(GraphType_Round, 1, 9, UI_Color_White, 10, 620, 160, 10);

    // 打符(单发)
    Char_State[1]   = UI_String_Init(0, 7, UI_Color_Cyan, 23, 4, 780, 125, "Rune");
    state_circle[1] = UI_Graph_Init(GraphType_Round, 1, 9, UI_Color_White, 10, 800, 160, 10);

    // 摩擦轮
    Char_State[2]   = UI_String_Init(0, 7, UI_Color_Cyan, 23, 4, 1000, 125, "Friction");
    state_circle[2] = UI_Graph_Init(GraphType_Round, 1, 9, UI_Color_White, 10, 1100, 160, 10);

    // 电容
    Char_State[3]   = UI_String_Init(0, 7, UI_Color_Cyan, 23, 4, 1250, 125, "Cap");
    state_circle[3] = UI_Graph_Init(GraphType_Round, 1, 9, UI_Color_White, 10, 1300, 160, 10);

    // 电容电压
    Char_State[4] = UI_String_Init(0, 7, UI_Color_Cyan, 20, 3, 25, 820, "Voltage");
    Cap_voltage   = UI_Graph_Init(GraphType_Float, 2, 0, UI_Color_Green, 20, 3, 3, 180, 820, (ui_cmd_recv.SuperCap_voltage) * 1000);

    // 底盘控制功率
    Char_State[5]      = UI_String_Init(0, 7, UI_Color_Cyan, 20, 3, 25, 780, "Chassis_Ctrl_Power");
    Chassis_Ctrl_Power = UI_Graph_Init(GraphType_Float, 2, 0, UI_Color_Green, 20, 3, 3, 400, 780, (ui_cmd_recv.Chassis_Ctrl_power) * 1000);

    // 电容吸收功率
    Char_State[6]    = UI_String_Init(0, 7, UI_Color_Cyan, 20, 3, 25, 740, "Cap_Absorb_Power");
    Cap_Absorb_Power = UI_Graph_Init(GraphType_Float, 2, 0, UI_Color_Green, 20, 3, 3, 400, 740, (ui_cmd_recv.Cap_absorb_power_limit) * 1000);

    // 底盘功率上限
    Char_State[7]       = UI_String_Init(0, 7, UI_Color_Cyan, 20, 3, 25, 700, "Chassis_Power_Limit");
    Chassis_Power_Limit = UI_Graph_Init(GraphType_Float, 2, 0, UI_Color_Green, 20, 3, 3, 400, 700, (ui_cmd_recv.Chassis_power_limit) * 1000);

    // 枪口热量
    Char_State[8]    = UI_String_Init(0, 7, UI_Color_Cyan, 20, 3, 25, 660, "Local_Heat");
    Shoot_Local_Heat = UI_Graph_Init(GraphType_Float, 2, 0, UI_Color_Green, 20, 3, 3, 250, 660, (ui_cmd_recv.Shooter_heat) * 1000);

    // 热量上限
    Char_State[9] = UI_String_Init(0, 7, UI_Color_Cyan, 20, 3, 25, 620, "Heat_Limit");
    Heat_Limit    = UI_Graph_Init(GraphType_Float, 2, 0, UI_Color_Green, 20, 3, 3, 250, 620, (ui_cmd_recv.Heat_Limit) * 1000);
}

void UI_Init()
{
    ui_pub = PubRegister("ui_feed", sizeof(UI_Upload_Data_s));
    ui_sub = SubRegister("ui_cmd", sizeof(UI_Cmd_s));

    UI_StaticInit();
}

void UIDynamicRefresh()
{
    SubGetMessage(ui_sub, (void *)&ui_cmd_recv);

    if (ui_cmd_recv.ui_send_flag == 0) {
        UI_StaticInit();
    }

    // 小陀螺
    if (ui_cmd_recv.chassis_mode == CHASSIS_ROTATE) {
        state_circle[0]->color = UI_Color_Green;
    } else {
        state_circle[0]->color = UI_Color_White;
    }
    // 打符(单发)
    if (ui_cmd_recv.rune_mode == 1) {
        state_circle[1]->color = UI_Color_Green;
    } else {
        state_circle[1]->color = UI_Color_White;
    }
    // 摩擦轮
    if (ui_cmd_recv.friction_mode == FRICTION_ON) {
        state_circle[2]->color = UI_Color_Green;
    } else {
        state_circle[2]->color = UI_Color_White;
    }
    // 电容&电容电压
    if (ui_cmd_recv.SuperCap_mode == SUPERCAP_PMOS_OPEN) {
        state_circle[3]->color         = UI_Color_Green;
        Cap_voltage->param.Float.value = (ui_cmd_recv.SuperCap_voltage) * 1000;
        Cap_voltage->color             = UI_Color_Orange;

    } else {
        state_circle[3]->color         = UI_Color_White;
        Cap_voltage->param.Float.value = (ui_cmd_recv.SuperCap_voltage) * 1000;
        Cap_voltage->color             = UI_Color_Green;
    }
    // 底盘功率
    Chassis_Ctrl_Power->param.Float.value = (ui_cmd_recv.Chassis_Ctrl_power) * 1000;

    // 电容吸收功率
    Cap_Absorb_Power->param.Float.value = (ui_cmd_recv.Cap_absorb_power_limit) * 1000;

    // 底盘功率上限
    Chassis_Power_Limit->param.Float.value = (ui_cmd_recv.Chassis_power_limit) * 1000;

    // 本地热量
    Shoot_Local_Heat->param.Float.value = (ui_cmd_recv.Shooter_heat) * 1000;

    // 热量上限
    Heat_Limit->param.Float.value = (ui_cmd_recv.Heat_Limit) * 1000;
    if (ui_cmd_recv.Heat_Limit - ui_cmd_recv.Shooter_heat < 20) {
        Heat_Limit->color       = UI_Color_Purplish_red;
        Shoot_Local_Heat->color = UI_Color_Purplish_red;
    } else {
        Heat_Limit->color       = UI_Color_Cyan;
        Shoot_Local_Heat->color = UI_Color_Cyan;
    }

    // 动态UI发送
    UI_Graph_Refresh();
    UI_String_Refresh();

    PubPushMessage(ui_pub, (void *)&ui_feedback_data);
}