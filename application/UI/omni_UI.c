#include "omni_UI.h"
#include "rm_referee.h"
#include "referee_protocol.h"
#include "referee_UI.h"
#include "string.h"
#include "crc_ref.h"
#include "stdio.h"
#include "rm_referee.h"
#include "message_center.h"
#include "robot_cmd.h"
#include "gimbal.h"
#include "dji_motor.h"
#include "super_cap.h"

extern referee_info_t referee_info;                         // 裁判系统数据
Referee_Interactive_info_t Referee_Interactive_info; // 绘制UI所需的数据
extern SuperCapInstance cap;
extern uint8_t auto_rune;   // 自瞄打符标志位
uint8_t Super_condition;    // 超电的开关状态
float Super_condition_volt; // 超电的电压

uint8_t UI_Seq;                    // 包序号，供整个referee文件使用
Graph_Data_t UI_shoot_line[10];    // 射击准线
Graph_Data_t UI_Deriction_line[4]; // 射击准线
Graph_Data_t UI_Energy[3];         // 电容能量条
Graph_Data_t UI_Rectangle[10];     // 矩形
Graph_Data_t UI_Circle_t[10];      // 圆形
Graph_Data_t UI_Arco_t[10];        // 圆弧
Graph_Data_t UI_Number_t[10];      // 数字
String_Data_t UI_State_sta[10];    // 机器人状态,静态只需画一次
String_Data_t UI_State_dyn[6];     // 机器人状态,动态先add才能change

char Send_Once_Flag = 0; // 初始化标志
uint32_t Rect_De[4] = {1540, 555, 1660, 645};
int16_t AIM_Rect_X, AIM_Rect_Y; // 自瞄框中心点的坐标信息
int16_t AIM_Rect_half_length = 50;
int16_t AIM_Rec_Color;
static uint32_t L_CRC_location[10] = {631, 1, 863, 347};
static uint32_t R_CRC_location[10] = {1283, 1, 1070, 347};
// static char char_data[20];

void UI_Init()
{
    // const float arc                 = 45.0f; // 弧长
    // const uint16_t Mechangle_offset = 10546;
    // float mid_point_angle           = fmod(720.0f - (YAW_CHASSIS_ALIGN_ECD - Mechangle_offset) * (360.0f / 8192.0f), 360.0f);
    // float angle_start               = fmod(mid_point_angle + 360.0f - arc / 2.0f, 360.0f);
    // float angle_end                 = fmod(mid_point_angle + arc / 2.0f, 360.0f);

    DeterminRobotID();

    // if (Send_Once_Flag == 0) {

    //     Send_Once_Flag = 1;

    // 清空UI
    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 9);

    // 射击线
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2, SCREEN_WIDTH / 2, SCREEN_LENGTH / 2, SCREEN_WIDTH / 2 - 500);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 80, SCREEN_WIDTH / 2 - 90, SCREEN_LENGTH / 2 + 80, SCREEN_WIDTH / 2 - 90);
    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 40, SCREEN_WIDTH / 2 - 220, SCREEN_LENGTH / 2 + 40, SCREEN_WIDTH / 2 - 220);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2, SCREEN_WIDTH / 2 + 20, SCREEN_LENGTH / 2, SCREEN_WIDTH / 2 - 210);
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 90, SCREEN_WIDTH / 2 - 40, SCREEN_LENGTH / 2 + 90, SCREEN_WIDTH / 2 - 40);
    UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 70, SCREEN_WIDTH / 2 - 120, SCREEN_LENGTH / 2 + 70, SCREEN_WIDTH / 2 - 120);
    // 位置标定线
    UILineDraw(&UI_Deriction_line[0], "sq0", UI_Graph_ADD, 9, UI_Color_White, 1, L_CRC_location[0], L_CRC_location[1], L_CRC_location[2], L_CRC_location[3]);
    UILineDraw(&UI_Deriction_line[1], "sq1", UI_Graph_ADD, 9, UI_Color_White, 1, R_CRC_location[0], R_CRC_location[1], R_CRC_location[2], R_CRC_location[3]);
    // 小陀螺
    // sprintf(friction_mode.show_Data, "BIU ON");
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 660, 100, "Rotate");
    UICharRefresh(&referee_info.referee_id, UI_State_sta[0]);
    UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_ADD, 9, UI_Color_White, 10, 700, 160, 8);
    // 打符
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 900, 100, "Rune");
    UICharRefresh(&referee_info.referee_id, UI_State_sta[1]);
    UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_ADD, 9, UI_Color_White, 10, 940, 160, 8);
    // 摩擦轮
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 1160, 100, "Fric");
    UICharRefresh(&referee_info.referee_id, UI_State_sta[2]);
    UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_ADD, 9, UI_Color_White, 10, 1180, 160, 8);
    // 电容
    UICircleDraw(&UI_Circle_t[3], "sc2", UI_Graph_ADD, 9, UI_Color_White, 10, 1300, 160, 8);
    // UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 80, 800, "SuperCap");
    // UICharRefresh(&referee_info.referee_id, UI_State_sta[3]);
    // UIRectangleDraw(&UI_Rectangle[1], "sr1", UI_Graph_ADD, 9, UI_Color_Yellow, 1, 80, 710, 280, 730);
    // UILineDraw(&UI_Energy[1], "sn1", UI_Graph_ADD, 8, UI_Color_Green, 20, 80, 720, (cap.cap_msg_s.CapVot - 12) / 12 * 200 + 80, 720);

    // 射击线
    UIGraphRefresh(&referee_info.referee_id, 7, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4], UI_shoot_line[5], UI_Deriction_line[0]);
    // 将位置标定线，小陀螺，弹舱盖，摩擦轮，电容一共7个图形打包一块发
    UIGraphRefresh(&referee_info.referee_id, 5, UI_Deriction_line[1], UI_Circle_t[0], UI_Circle_t[1], UI_Circle_t[2], UI_Circle_t[3]);
    // }

    // else {

    // }
}

void UITask()
{
    // 底盘模式
    if (Referee_Interactive_info.chassis_mode == CHASSIS_ROTATE) {
        UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_Change, 9, UI_Color_Green, 10, 700, 160, 8);
    } else if (Referee_Interactive_info.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
        UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_Change, 9, UI_Color_White, 10, 700, 160, 8);
    }
    // 摩擦轮
    if (Referee_Interactive_info.friction_mode == FRICTION_ON) // 摩擦轮开启模式下
    {
        UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_Green, 10, 1180, 160, 8);
    } else if (Referee_Interactive_info.friction_mode == FRICTION_OFF) // 摩擦轮关闭
    {
        UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 10, 1180, 160, 8);
    }
    // 电容
    UIRectangleDraw(&UI_Rectangle[1], "sr1", UI_Graph_ADD, 9, UI_Color_Yellow, 1, 80, 710, 280, 730);
    UILineDraw(&UI_Energy[1], "sn1", UI_Graph_Change, 8, UI_Color_Green, 20, 80, 720, (cap.cap_msg_s.CapVot - 12) / 12 * 200 + 80, 720);
    // 打符
    if (Referee_Interactive_info.auto_rune == 1) {
        UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 9, UI_Color_Purplish_red, 26, 2, 880, 820, "Rune_ON!!!");
        UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_Purplish_red, 10, 940, 160, 8);
    } else {
        UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_Del, 9, UI_Color_Purplish_red, 26, 2, 880, 820, "Rune_ON!!!");
        UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_White, 10, 940, 160, 8);
    }
    UICharRefresh(&referee_info.referee_id, UI_State_sta[5]);

    // 动态图形正好7个，打包一块发
    UIGraphRefresh(&referee_info.referee_id, 1, UI_Rectangle[2]);
    UIGraphRefresh(&referee_info.referee_id, 7, UI_Circle_t[0], UI_Circle_t[2], UI_Rectangle[1], UI_Energy[1], UI_Circle_t[1], UI_Arco_t[0], UI_Arco_t[1]);
}