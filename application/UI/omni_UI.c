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

static Publisher_t *ui_pub;
static Subscriber_t *ui_sub;
static UI_Cmd_s ui_cmd_recv;
static UI_Upload_Data_s ui_feedback_data;

referee_info_t *referee_data_for_ui;

uint8_t UI_rune; // 自瞄打符标志位

uint8_t UI_Seq;                // 包序号，供整个referee文件使用
Graph_Data_t UI_shoot_line[7]; // 射击准线
Graph_Data_t UI_Circle_t[10];  // 圆形

static void UI_StaticRefresh()
{
    // 清空UI
    UIDelete(&referee_data_for_ui->referee_id, UI_Data_Del_ALL, 0);

    // 射击线
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 9, UI_Color_Green, 1, SCREEN_LENGTH / 2, SCREEN_WIDTH / 2, SCREEN_LENGTH / 2, SCREEN_WIDTH / 2 - 500);
    // UIGraphRefresh(&referee_info.referee_id, 1, UI_shoot_line[0]);
    // UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 80, SCREEN_WIDTH / 2 - 90, SCREEN_LENGTH / 2 + 80, SCREEN_WIDTH / 2 - 90);
    // UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 40, SCREEN_WIDTH / 2 - 220, SCREEN_LENGTH / 2 + 40, SCREEN_WIDTH / 2 - 220);
    // UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2, SCREEN_WIDTH / 2 + 20, SCREEN_LENGTH / 2, SCREEN_WIDTH / 2 - 210);
    // UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 90, SCREEN_WIDTH / 2 - 40, SCREEN_LENGTH / 2 + 90, SCREEN_WIDTH / 2 - 40);
    // UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 70, SCREEN_WIDTH / 2 - 120, SCREEN_LENGTH / 2 + 70, SCREEN_WIDTH / 2 - 120);
    // // 位置标定线
    // UILineDraw(&UI_Deriction_line[0], "sq0", UI_Graph_ADD, 9, UI_Color_White, 1, L_CRC_location[0], L_CRC_location[1], L_CRC_location[2], L_CRC_location[3]);
    // UILineDraw(&UI_Deriction_line[1], "sq1", UI_Graph_ADD, 9, UI_Color_White, 1, R_CRC_location[0], R_CRC_location[1], R_CRC_location[2], R_CRC_location[3]);
    // // 小陀螺
    // // sprintf(friction_mode.show_Data, "BIU ON");
    // UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 660, 100, "Rotate");
    // UICharRefresh(&referee_info.referee_id, UI_State_sta[0]);
    // UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_ADD, 9, UI_Color_White, 10, 700, 160, 8);
    // // 打符
    // UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 900, 100, "Rune");
    // UICharRefresh(&referee_info.referee_id, UI_State_sta[1]);
    // UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_ADD, 9, UI_Color_White, 10, 940, 160, 8);
    // // 摩擦轮
    // UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 1160, 100, "Fric");
    // UICharRefresh(&referee_info.referee_id, UI_State_sta[2]);
    // UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_ADD, 9, UI_Color_White, 10, 1180, 160, 8);
    // // 电容
    // UICircleDraw(&UI_Circle_t[3], "sc2", UI_Graph_ADD, 9, UI_Color_White, 10, 1300, 160, 8);
    // // UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 80, 800, "SuperCap");
    // // UICharRefresh(&referee_info.referee_id, UI_State_sta[3]);
    // // UIRectangleDraw(&UI_Rectangle[1], "sr1", UI_Graph_ADD, 9, UI_Color_Yellow, 1, 80, 710, 280, 730);
    // // UILineDraw(&UI_Energy[1], "sn1", UI_Graph_ADD, 8, UI_Color_Green, 20, 80, 720, (cap.cap_msg_s.CapVot - 12) / 12 * 200 + 80, 720);

    // // 射击线
    UIGraphRefresh(&referee_data_for_ui->referee_id, 1, UI_shoot_line[0]);
    // UIGraphRefresh(&referee_info.referee_id, 7, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4], UI_shoot_line[5], UI_Deriction_line[0]);
    // // 将位置标定线，小陀螺，弹舱盖，摩擦轮，电容一共7个图形打包一块发
    // UIGraphRefresh(&referee_info.referee_id, 5, UI_Deriction_line[1], UI_Circle_t[0], UI_Circle_t[1], UI_Circle_t[2], UI_Circle_t[3]);
}

void UI_Init()
{
    ui_pub = PubRegister("ui_cmd", sizeof(UI_Cmd_s));
    ui_sub = SubRegister("ui_feed", sizeof(UI_Upload_Data_s));

    UI_StaticRefresh();
}

// static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data){

// }

void My_UIGraphRefresh()
{
    SubGetMessage(ui_sub, (void *)&ui_cmd_recv);
    if (ui_cmd_recv.ui_send_flag == 0) {
        UI_StaticRefresh();
    }
    // // 底盘模式
    // if (Referee_Interactive_info.chassis_mode == CHASSIS_ROTATE) {
    //     UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_Change, 9, UI_Color_Green, 10, 700, 160, 8);
    // } else if (Referee_Interactive_info.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
    //     UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_Change, 9, UI_Color_White, 10, 700, 160, 8);
    // }
    // // 摩擦轮
    // if (Referee_Interactive_info.friction_mode == FRICTION_ON) // 摩擦轮开启模式下
    // {
    //     UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_Green, 10, 1180, 160, 8);
    // } else if (Referee_Interactive_info.friction_mode == FRICTION_OFF) // 摩擦轮关闭
    // {
    //     UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 10, 1180, 160, 8);
    // }
    // // 电容
    // UIRectangleDraw(&UI_Rectangle[1], "sr1", UI_Graph_ADD, 9, UI_Color_Yellow, 1, 80, 710, 280, 730);
    // UILineDraw(&UI_Energy[1], "sn1", UI_Graph_Change, 8, UI_Color_Green, 20, 80, 720, (cap.cap_msg_s.CapVot - 12) / 12 * 200 + 80, 720);
    // // 打符
    // if (Referee_Interactive_info.auto_rune == 1) {
    //     UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 9, UI_Color_Purplish_red, 26, 2, 880, 820, "Rune_ON!!!");
    //     UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_Purplish_red, 10, 940, 160, 8);
    // } else {
    //     UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_Del, 9, UI_Color_Purplish_red, 26, 2, 880, 820, "Rune_ON!!!");
    //     UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_White, 10, 940, 160, 8);
    // }
    // UICharRefresh(&referee_info.referee_id, UI_State_sta[5]);

    // // 动态图形正好7个，打包一块发
    // UIGraphRefresh(&referee_info.referee_id, 1, UI_Rectangle[2]);
    // UIGraphRefresh(&referee_info.referee_id, 7, UI_Circle_t[0], UI_Circle_t[2], UI_Rectangle[1], UI_Energy[1], UI_Circle_t[1], UI_Arco_t[0], UI_Arco_t[1]);
    PubPushMessage(ui_pub, (void *)&ui_feedback_data);
}