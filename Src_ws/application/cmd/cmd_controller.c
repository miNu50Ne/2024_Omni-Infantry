/**
 * @file cmd_controller.c
 * @author miNu50Ne (minu50ne@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-07
 *
 * @copyright Copyright (c) 2024
 *
 */
// app
#include "robot_def.h"
#include "cmd_controller.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "referee_init.h"

#include "ramp.h"
// bsp
#include "bsp_log.h"
#include <math.h>
#include <string.h>

static Publisher_t *chassis_cmd_pub;             // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub;           // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Publisher_t *ui_cmd_pub;        // UI控制消息发布者
static Subscriber_t *ui_feed_sub;      // UI反馈信息订阅者
static UI_Cmd_s ui_cmd_send;           // 传递给UI的控制信息
static UI_Upload_Data_s ui_fetch_data; // 从UI获取的反馈信息

static Publisher_t *master_cmd_pub;             // 上位机控制消息发布者
static Subscriber_t *master_feed_sub;           // 上位机反馈消息订阅者
static Master_Cmd_s *master_cmd_send;           // 传递给上位机的控制信息
static Master_Upload_Data_s *master_fetch_data; // 从上位机获得的反馈信息

static RC_ctrl_t *rc_data;           // 遥控器数据,初始化时返回
static referee_info_t *referee_data; // 用于获取裁判系统的数据
static CmdInstance cmd_media_param;  // 控制中介变量

void CmdDeviceInit()
{
    rc_data      = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    referee_data = RefereeHardwareInit(&huart6); // 裁判系统初始化,会同时初始化UI
}

void CmdParamInit()
{
#if PITCH_FEED_TYPE
    gimbal_cmd_send.pitch = 0;
#else
    gimbal_cmd_send.pitch = PTICH_HORIZON_ANGLE;
#endif
    shoot_cmd_send.shoot_mode = SHOOT_OFF; // 初始化后发射机构失能

    memset(&cmd_media_param, 0, sizeof(CmdInstance));
    ramp_init(cmd_media_param.fb_ramp, RAMP_TIME);
    ramp_init(cmd_media_param.lr_ramp, RAMP_TIME);

    cmd_media_param.UI_SendFlag = 1;
    cmd_media_param.auto_rune   = 0;
}

void CmdMsgInit()
{
    gimbal_cmd_pub   = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub  = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub    = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub   = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    master_cmd_pub   = PubRegister("master_cmd", sizeof(Master_Cmd_s));
    master_feed_sub  = SubRegister("master_feed", sizeof(Master_Upload_Data_s));
    ui_cmd_pub       = PubRegister("ui_cmd", sizeof(UI_Cmd_s));
    ui_feed_sub      = SubRegister("ui_feed", sizeof(UI_Upload_Data_s));
}

void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_data->referee_id.Robot_Color       = referee_data->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_data->referee_id.Cilent_ID         = 0x0100 + referee_data->GameRobotState.robot_id; // 计算客户端ID
    referee_data->referee_id.Robot_ID          = referee_data->GameRobotState.robot_id;          // 计算机器人ID
    referee_data->referee_id.Receiver_Robot_ID = 0;
}

void CalcOffsetAngle()
{
    // 从云台获取的当前yaw电机单圈角度
    float angle = gimbal_fetch_data.yaw_motor_single_round_angle;
    // 云台yaw轴当前角度
    float gimbal_yaw_current_angle = gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET];
    // 云台yaw轴目标角度
    float gimbal_yaw_set_angle          = cmd_media_param.yaw_control;
    chassis_cmd_send.gimbal_error_angle = gimbal_yaw_set_angle - gimbal_yaw_current_angle; // 云台误差角

#if YAW_ECD_GREATER_THAN_4096 // 如果大于180度
    if (angle < 180.0f + YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#else // 小于180度
    if (angle >= YAW_ALIGN_ANGLE - 180.0f && angle <= YAW_ALIGN_ANGLE + 180.0f) {
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    } else {
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    }
#endif
    chassis_cmd_send.offset_angle = chassis_cmd_send.offset_angle + chassis_cmd_send.gimbal_error_angle;
}

void PitchAngleLimit()
{
    float limit_min, limit_max;
#if PITCH_INS_FEED_TYPE
    limit_min = PITCH_LIMIT_ANGLE_DOWN * DEGREE_2_RAD;
    limit_max = PITCH_LIMIT_ANGLE_UP * DEGREE_2_RAD;
#else
    limit_min = PITCH_LIMIT_ANGLE_DOWN;
    limit_max = PITCH_LIMIT_ANGLE_UP;
#endif

#if PITCH_ECD_UP_ADD // 云台抬升,反馈值增
    if (current > limit_max)
        current = limit_max;
    if (current < limit_min)
        current = limit_min;
#else
    if (cmd_media_param.pitch_control < limit_max)
        cmd_media_param.pitch_control = limit_max;
    if (cmd_media_param.pitch_control > limit_min)
        cmd_media_param.pitch_control = limit_min;
#endif
}

void AutoControlSwitch()
{
    if (master_fetch_data->rec_pitch == 0 && master_fetch_data->rec_yaw == 0) {
        cmd_media_param.yaw_control += YAW_K * (float)rc_data[TEMP].rc.rocker_l_;
        cmd_media_param.pitch_control += PITCH_K * (float)rc_data[TEMP].rc.rocker_l1;
    } else {
        cmd_media_param.yaw_control   = master_fetch_data->rec_yaw;
        cmd_media_param.pitch_control = master_fetch_data->rec_pitch;
    }
}

void YawControlProcess()
{
    if (cmd_media_param.yaw_control - gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET] > 180) {
        cmd_media_param.yaw_control -= 360;
    } else if (cmd_media_param.yaw_control - gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET] < -180) {
        cmd_media_param.yaw_control += 360;
    }
}

void ShootHeatControl()
{
    if (shoot_cmd_send.friction_mode == FRICTION_OFF) {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    float rate_coef = 0;
    if (cmd_media_param.heat_coef == 1)
        rate_coef = 1;
    else if (cmd_media_param.heat_coef >= 0.8 && cmd_media_param.heat_coef < 1)
        rate_coef = 0.8;
    else if (cmd_media_param.heat_coef >= 0.6 && cmd_media_param.heat_coef < 0.8)
        rate_coef = 0.6;
    else if (cmd_media_param.heat_coef < 0.6)
        rate_coef = 0.4;
    cmd_media_param.heat_coef = ((referee_data->GameRobotState.shooter_id1_17mm_cooling_limit - referee_data->PowerHeatData.shooter_17mm_heat0 + rate_coef * referee_data->GameRobotState.shooter_id1_17mm_cooling_rate) * 1.0f) / (1.0f * referee_data->GameRobotState.shooter_id1_17mm_cooling_limit);
    // 新热量管理
    if (referee_data->GameRobotState.shooter_id1_17mm_cooling_limit - 40 + 30 * cmd_media_param.heat_coef - shoot_fetch_data.shooter_local_heat <= shoot_fetch_data.shooter_heat_control) {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

/**
 * @brief  紧急停止,双下
 *
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void emergencyhandler()
{
    gimbal_cmd_send.gimbal_mode              = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode            = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.SuperCap_flag_from_user = SUPER_USER_CLOSE;
    shoot_cmd_send.friction_mode             = FRICTION_OFF;
    shoot_cmd_send.load_mode                 = LOAD_STOP;
    shoot_cmd_send.shoot_mode                = SHOOT_OFF;
    memset(cmd_media_param.rc_mode, 1, sizeof(uint8_t));
    memset(cmd_media_param.rc_mode + 1, 0, sizeof(uint8_t) * 4);
    LOGERROR("[CMD] emergency stop!");
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void remotecontrolset()
{
    shoot_cmd_send.shoot_mode   = SHOOT_ON; // 发射机构常开
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    shoot_cmd_send.shoot_rate   = 30; // 射频默认30Hz

    if (rc_data[TEMP].rc.dial > 400) {
        chassis_cmd_send.SuperCap_flag_from_user = SUPER_USER_OPEN;
    } else {
        chassis_cmd_send.SuperCap_flag_from_user = SUPER_USER_CLOSE; // 默认关闭超电
    }

    switch (rc_data[TEMP].rc.switch_right) {
        case RC_SW_MID:
            if (cmd_media_param.rc_mode[CHASSIS_FREE] == 1) {
                chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
            }

            if (rc_data[TEMP].rc.switch_left == RC_SW_MID && chassis_cmd_send.chassis_mode == CHASSIS_NO_FOLLOW) {
                cmd_media_param.rc_mode[CHASSIS_ROTATION] = 1;
                cmd_media_param.rc_mode[CHASSIS_FOLLOW]   = 1;
            }

            if (chassis_cmd_send.chassis_mode == CHASSIS_ROTATE) {
                cmd_media_param.rc_mode[CHASSIS_ROTATION] = 0;
            }

            if (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
                cmd_media_param.rc_mode[CHASSIS_FOLLOW] = 0;
            }
            break;
        case RC_SW_DOWN:
            cmd_media_param.rc_mode[CHASSIS_FREE] = 0;
            if (cmd_media_param.rc_mode[CHASSIS_ROTATION] == 1) {
                if (rc_data[TEMP].rc.dial < -400) {
                    chassis_cmd_send.chassis_mode = CHASSIS_REVERSE_ROTATE;
                } else {
                    chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
                }
            } else {
                chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
            }
            break;
        case RC_SW_UP:
            cmd_media_param.rc_mode[CHASSIS_FREE] = 0;
            if (cmd_media_param.rc_mode[CHASSIS_FOLLOW] == 1) {
                chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            } else {
                chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
            }
            break;
        default:
            break;
    }

    switch (rc_data[TEMP].rc.switch_left) {
        case RC_SW_MID:
            if (rc_data[TEMP].rc.switch_right == RC_SW_MID) {
                // 摩擦轮
                if (cmd_media_param.rc_mode[SHOOT_FRICTION] == 0 && shoot_cmd_send.friction_mode == FRICTION_OFF) {
                    cmd_media_param.rc_mode[SHOOT_FRICTION] = 1;
                }
            }
            if (shoot_cmd_send.friction_mode == FRICTION_ON) {
                cmd_media_param.rc_mode[SHOOT_FRICTION] = 0;
            }
            break;
        case RC_SW_UP:
            if (cmd_media_param.rc_mode[SHOOT_FRICTION] == 1) {
                shoot_cmd_send.friction_mode = FRICTION_ON;
            } else {
                shoot_cmd_send.friction_mode = FRICTION_OFF;
            }
            break;
        case RC_SW_DOWN:
            if (rc_data[TEMP].rc.dial < -400) {
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            } else {
                shoot_cmd_send.load_mode = LOAD_1_BULLET;
            }
        default:
            break;
    }

    // 底盘参数
    chassis_cmd_send.vx = 70.0f * (float)rc_data[TEMP].rc.rocker_r_; // 水平方向
    chassis_cmd_send.vy = 70.0f * (float)rc_data[TEMP].rc.rocker_r1; // 竖直方向
    // 云台参数

    gimbal_cmd_send.yaw   = cmd_media_param.yaw_control;
    gimbal_cmd_send.pitch = cmd_media_param.pitch_control;
}

/**
 * @brief 键盘设定速度
 *
 */
static void chassisset()
{
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;

    // 底盘移动
    static float current_speed_x = 0;
    static float current_speed_y = 0;
    // 前后移动
    // 防止逃跑时关小陀螺按Ctrl进入慢速模式
    if (rc_data[TEMP].key[KEY_PRESS].w) {
        chassis_cmd_send.vy = (current_speed_y + (40000 - current_speed_y) * ramp_calc(cmd_media_param.fb_ramp)); // vx方向待测
    } else if (rc_data[TEMP].key[KEY_PRESS].s) {
        chassis_cmd_send.vy = (current_speed_y + (-40000 - current_speed_y) * ramp_calc(cmd_media_param.fb_ramp));
    } else {
        chassis_cmd_send.vy = 0;
        ramp_init(cmd_media_param.fb_ramp, RAMP_TIME);
    }

    // 左右移动
    if (rc_data[TEMP].key[KEY_PRESS].a) {
        chassis_cmd_send.vx = (current_speed_x + (40000 - current_speed_x) * ramp_calc(cmd_media_param.lr_ramp));
    } else if (rc_data[TEMP].key[KEY_PRESS].d) {
        chassis_cmd_send.vx = (current_speed_x + (-40000 - current_speed_x) * ramp_calc(cmd_media_param.lr_ramp));
    } else {
        chassis_cmd_send.vx = 0;
        ramp_init(cmd_media_param.lr_ramp, RAMP_TIME);
    }

    current_speed_x = chassis_cmd_send.vx;
    current_speed_y = chassis_cmd_send.vy;
}

/**
 * @brief 鼠标移动云台
 *
 */
static void gimbalset()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    // 按住鼠标右键且视觉识别到目标
    cmd_media_param.yaw_control -= rc_data[TEMP].mouse.x / 350.0f;
    cmd_media_param.pitch_control -= -rc_data[TEMP].mouse.y / 15500.0f;
    gimbal_cmd_send.yaw   = cmd_media_param.yaw_control;
    gimbal_cmd_send.pitch = cmd_media_param.pitch_control;
}

/**
 * @brief 键鼠设定机器人发射模式
 *
 */
static void shootset()
{
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.shoot_rate = 30; // 射频默认30Hz

    // 仅在摩擦轮开启时有效
    if (shoot_cmd_send.friction_mode == FRICTION_ON) {
        // 打弹，单击左键单发，长按连发
        if (rc_data[TEMP].mouse.press_l) {
            // 打符，单发
            if (cmd_media_param.auto_rune == 1) {
                shoot_cmd_send.load_mode = LOAD_1_BULLET;
            } else {
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            }
        } else {
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
    } else {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

/**
 * @brief 键盘处理模式标志位
 *
 */

static void keymodeset()
{
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 2) {
        case 1:
            if (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
                chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            break;
        case 0:
            if (chassis_cmd_send.chassis_mode == CHASSIS_ROTATE)
                chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_V] % 2) {
        case 1:
            if (shoot_cmd_send.friction_mode != FRICTION_ON)
                shoot_cmd_send.friction_mode = FRICTION_ON;
            break;
        case 0:
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].r) {
        case 1:
            if (cmd_media_param.UI_SendFlag == 1) {
                cmd_media_param.UI_SendFlag = 0;
            }
            break;
        case 0:
            cmd_media_param.UI_SendFlag = 1;
            break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].ctrl) {
        case 1:
            cmd_media_param.auto_rune = 1;
            break;
        case 0:
            cmd_media_param.auto_rune = 0;
            break;
        default:
            break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) {
        case 1:
            chassis_cmd_send.SuperCap_flag_from_user = SUPER_USER_OPEN;
            break;
        case 0:
            chassis_cmd_send.SuperCap_flag_from_user = SUPER_USER_CLOSE;
            break;
    }
}

/**
 * @brief 机器人复位函数，按下Ctrl+Shift+r
 *
 *
 */
static void robotreset()
{
    if (rc_data[TEMP].key[KEY_PRESS].shift && rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].r) {
        osDelay(1000);
        __set_FAULTMASK(1);
        NVIC_SystemReset(); // 软件复位
    }
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void mousekeyset()
{
    chassisset();
    gimbalset();
    shootset();
    keymodeset();
    robotreset(); // 机器人复位处理
}

extern referee_info_t *referee_data_for_ui;

void CmdModeSet()
{
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_up(rc_data[TEMP].rc.switch_left) && (switch_is_down(rc_data[TEMP].rc.switch_right))) // 遥控器拨杆右[上]左[下],键鼠控制
        mousekeyset();
    else if (RC_LOST || (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right))) {
        emergencyhandler(); // 调试/疯车时急停
    } else {
        remotecontrolset();
    }
    // chassis_cmd_send.chassis_cmd_velocity_vector = sqrtf(powf(chassis_cmd_send.vx * DEGREE_2_RAD * RADIUS_WHEEL * 10e3, 2) + powf(chassis_cmd_send.vy * DEGREE_2_RAD * RADIUS_WHEEL * 10e3, 2));
}

void CmdMsgComm()
{
    // 从其他应用获取回传数据
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(ui_feed_sub, &ui_fetch_data);
    SubGetMessage(master_feed_sub, &master_fetch_data);

    // 推送消息
    // chassis
    memcpy(&chassis_cmd_send.chassis_power, &referee_data->PowerHeatData.chassis_power, sizeof(float));
    memcpy(&chassis_cmd_send.power_buffer, &referee_data->PowerHeatData.chassis_power_buffer, sizeof(uint16_t));
    memcpy(&chassis_cmd_send.level, &referee_data->GameRobotState.robot_level, sizeof(uint8_t));
    memcpy(&chassis_cmd_send.power_limit, &referee_data->GameRobotState.chassis_power_limit, sizeof(uint16_t));
    // memcpy

    // shoot
    memcpy(&shoot_cmd_send.shooter_heat_cooling_rate, &referee_data->GameRobotState.shooter_id1_17mm_cooling_rate, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.shooter_referee_heat, &referee_data->PowerHeatData.shooter_17mm_heat0, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.shooter_cooling_limit, &referee_data->GameRobotState.shooter_id1_17mm_cooling_limit, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.bullet_speed, &referee_data->ShootData.bullet_speed, sizeof(float));

    // UI
    // memcpy(referee_data_for_ui, referee_data, sizeof(referee_info_t));
    referee_data_for_ui = referee_data;
    memcpy(&ui_cmd_send.ui_send_flag, &cmd_media_param.UI_SendFlag, sizeof(uint8_t));
    memcpy(&ui_cmd_send.chassis_mode, &chassis_cmd_send.chassis_mode, sizeof(chassis_mode_e));
    memcpy(&ui_cmd_send.chassis_attitude_angle, &gimbal_fetch_data.yaw_motor_single_round_angle, sizeof(uint16_t));
    memcpy(&ui_cmd_send.friction_mode, &shoot_cmd_send.friction_mode, sizeof(friction_mode_e));
    memcpy(&ui_cmd_send.rune_mode, &cmd_media_param.auto_rune, sizeof(uint8_t));
    memcpy(&ui_cmd_send.SuperCap_mode, &chassis_fetch_data.CapFlag_open_from_real, sizeof(uint8_t));
    memcpy(&ui_cmd_send.SuperCap_voltage, &chassis_fetch_data.cap_voltage, sizeof(float));
    memcpy(&ui_cmd_send.Chassis_power_limit, &referee_data->GameRobotState.chassis_power_limit, sizeof(uint16_t));
    memcpy(&ui_cmd_send.Shooter_heat, &shoot_fetch_data.shooter_local_heat, sizeof(float));
    memcpy(&ui_cmd_send.Heat_Limit, &referee_data->GameRobotState.shooter_id1_17mm_cooling_limit, sizeof(uint16_t));

    static uint8_t frame_head[] = {0xAF, 0x32, 0x00, 0x12};
    memcpy(master_cmd_send->frame_head, frame_head, 4);
    memcpy(master_cmd_send->ins_quat, gimbal_fetch_data.gimbal_imu_data->INS_data.INS_quat, sizeof(float) * 4);
    memcpy(&master_cmd_send->robot_id, &referee_data->GameRobotState.robot_id, sizeof(uint8_t));
    memcpy(&master_cmd_send->rune_mode, &cmd_media_param.auto_rune, sizeof(uint8_t));

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(ui_cmd_pub, (void *)&ui_cmd_send);
    PubPushMessage(master_cmd_pub, (void *)&master_cmd_send);
}
