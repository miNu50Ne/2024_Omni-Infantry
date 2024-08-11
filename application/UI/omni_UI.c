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

uint8_t UI_Seq;                      // 包序号，供整个referee文件使用
static Graph_Data_t shoot_line[7];   // 射击准线
static Graph_Data_t state_circle[4]; // 圆形
static String_Data_t Char_State[6];  // 字符串

static Graph_Data_t Cap_voltage;      // 电容电压
static Graph_Data_t Shoot_Local_Heat; // 射击本地热量

static void UI_StaticInit()
{
    // 清空UI
    UIDelete(&referee_data_for_ui->referee_id, UI_Data_Del_ALL, 0);

    // 射击准心
    UILineDraw(&shoot_line[0], "ol0", UI_Graph_ADD, 9, UI_Color_Green, 1, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 + 5, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 44);
    UILineDraw(&shoot_line[1], "ol1", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 75, SCREEN_WIDTH / 2 - 50, SCREEN_LENGTH / 2 - 30, SCREEN_WIDTH / 2 - 50);
    UILineDraw(&shoot_line[2], "ol2", UI_Graph_ADD, 9, UI_Color_Green, 1, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 56, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 105);
    UILineDraw(&shoot_line[3], "ol3", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 20, SCREEN_WIDTH / 2 - 50, SCREEN_LENGTH / 2 + 25, SCREEN_WIDTH / 2 - 50);

    UILineDraw(&shoot_line[4], "ol4", UI_Graph_ADD, 9, UI_Color_Green, 1, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 49, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 51);

    UILineDraw(&shoot_line[5], "ol5", UI_Graph_ADD, 9, UI_Color_White, 2, 540, 150, 700, 300);
    UILineDraw(&shoot_line[6], "ol6", UI_Graph_ADD, 9, UI_Color_White, 2, 1380, 150, 1220, 300);

    // 小陀螺
    sprintf(Char_State[0].show_Data, "Rotate");
    UICharDraw(&Char_State[0], "sc0", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 580, 125, "Rotate");
    UICharRefresh(&referee_data_for_ui->referee_id, Char_State[0]);

    UICircleDraw(&state_circle[0], "oc0", UI_Graph_ADD, 9, UI_Color_White, 10, 620, 160, 10);
    // 打符(单发)
    sprintf(Char_State[1].show_Data, "Rune");
    UICharDraw(&Char_State[1], "sc1", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 780, 125, "Rune");
    UICharRefresh(&referee_data_for_ui->referee_id, Char_State[1]);

    UICircleDraw(&state_circle[1], "oc1", UI_Graph_ADD, 9, UI_Color_White, 10, 800, 160, 10);
    // 摩擦轮
    sprintf(Char_State[2].show_Data, "Friction");
    UICharDraw(&Char_State[2], "sc2", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 1000, 125, "Friction");
    UICharRefresh(&referee_data_for_ui->referee_id, Char_State[2]);

    UICircleDraw(&state_circle[2], "oc2", UI_Graph_ADD, 9, UI_Color_White, 10, 1100, 160, 10);
    // 电容
    sprintf(Char_State[3].show_Data, "Cap");
    UICharDraw(&Char_State[3], "sc3", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 1270, 125, "Cap");
    UICharRefresh(&referee_data_for_ui->referee_id, Char_State[3]);

    UICircleDraw(&state_circle[3], "oc3", UI_Graph_ADD, 9, UI_Color_White, 10, 1300, 160, 10);

    // 电容电压
    sprintf(Char_State[4].show_Data, "Voltage");
    UICharDraw(&Char_State[4], "sc4", UI_Graph_ADD, 7, UI_Color_Orange, 20, 3, 25, 820, "Voltage");
    UICharRefresh(&referee_data_for_ui->referee_id, Char_State[4]);

    UIFloatDraw(&Cap_voltage, "of0", UI_Graph_ADD, 9, UI_Color_Green, 20, 3, 3, 180, 820, (ui_cmd_recv.SuperCap_voltage) * 1000);

    // 枪口热量
    sprintf(Char_State[5].show_Data, "Shoot_Heat");
    UICharDraw(&Char_State[5], "sc5", UI_Graph_ADD, 7, UI_Color_Orange, 20, 3, 25, 780, "Shoot_Heat");
    UICharRefresh(&referee_data_for_ui->referee_id, Char_State[5]);

    UIFloatDraw(&Shoot_Local_Heat, "of1", UI_Graph_ADD, 9, UI_Color_Green, 20, 3, 3, 400, 780, (ui_cmd_recv.Shooter_heat) * 1000);

    // 发送
    UIGraphRefresh(&referee_data_for_ui->referee_id, 7, shoot_line[0], shoot_line[1], shoot_line[2], shoot_line[3], shoot_line[4], shoot_line[5], shoot_line[6]);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 7, state_circle[0], state_circle[1], state_circle[2], state_circle[3], Cap_voltage, Shoot_Local_Heat);
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
        UICircleDraw(&state_circle[0], "oc0", UI_Graph_Change, 9, UI_Color_Green, 10, 620, 160, 10);
    } else {
        UICircleDraw(&state_circle[0], "oc0", UI_Graph_Change, 9, UI_Color_White, 10, 620, 160, 10);
    }
    // 打符(单发)
    if (ui_cmd_recv.rune_mode == 1) {
        UICircleDraw(&state_circle[1], "oc1", UI_Graph_Change, 9, UI_Color_Green, 10, 800, 160, 10);
    } else {
        UICircleDraw(&state_circle[1], "oc1", UI_Graph_Change, 9, UI_Color_White, 10, 800, 160, 10);
    }
    // 摩擦轮
    if (ui_cmd_recv.friction_mode == FRICTION_ON) {
        UICircleDraw(&state_circle[2], "oc2", UI_Graph_Change, 9, UI_Color_Green, 10, 1100, 160, 10);
    } else {
        UICircleDraw(&state_circle[2], "oc2", UI_Graph_Change, 9, UI_Color_White, 10, 1100, 160, 10);
    }
    // 电容&电容电压
    if (ui_cmd_recv.SuperCap_mode == SUPERCAP_PMOS_OPEN) {
        UICircleDraw(&state_circle[3], "oc3", UI_Graph_Change, 9, UI_Color_Green, 10, 1300, 160, 10);
        UIFloatDraw(&Cap_voltage, "of0", UI_Graph_Change, 9, UI_Color_Orange, 20, 3, 3, 180, 820, (ui_cmd_recv.SuperCap_voltage) * 1000);
    } else {
        UICircleDraw(&state_circle[3], "oc3", UI_Graph_Change, 9, UI_Color_White, 10, 1300, 160, 10);
        UIFloatDraw(&Cap_voltage, "of0", UI_Graph_Change, 9, UI_Color_Purplish_red, 20, 3, 3, 180, 820, (ui_cmd_recv.SuperCap_voltage) * 1000);
    }

    if (ui_cmd_recv.Heat_Limit - ui_cmd_recv.Shooter_heat > 20) {
        // 本地热量
        UIFloatDraw(&Shoot_Local_Heat, "of1", UI_Graph_ADD, 9, UI_Color_Green, 20, 3, 3, 400, 780, (ui_cmd_recv.Shooter_heat) * 1000);
    } else {
        UIFloatDraw(&Shoot_Local_Heat, "of1", UI_Graph_ADD, 9, UI_Color_Purplish_red, 20, 3, 3, 400, 780, (ui_cmd_recv.Shooter_heat) * 1000);
    }

    // 动态UI发送
    UIGraphRefresh(&referee_data_for_ui->referee_id, 5, state_circle[0], state_circle[1], state_circle[2], state_circle[3], Cap_voltage);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 1, Shoot_Local_Heat);

    PubPushMessage(ui_pub, (void *)&ui_feedback_data);
}