// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "omni_UI.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "referee_UI.h"
#include "referee_init.h"

#include "tool.h"
#include "super_cap.h"
#include "AHRS_MiddleWare.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#define RC_LOST (rc_data[TEMP].rc.switch_left == 0 && rc_data[TEMP].rc.switch_right == 0)

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#if PITCH_FEED_TYPE                                                  // Pitch电机反馈数据源为陀螺仪
#define PTICH_HORIZON_ANGLE 0                                        // PITCH水平时电机的角度
#if PITCH_ECD_UP_ADD
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#else
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif
#else                                                                   // PITCH电机反馈数据源为编码器
#define PTICH_HORIZON_ANGLE    (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // PITCH水平时电机的角度,0-360
#define PITCH_LIMIT_ANGLE_UP   (PITCH_POS_MAX_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (PITCH_POS_MIN_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回

HostInstance *host_instance; // 上位机接口

// 这里的四元数以wxyz的顺序
static uint8_t vision_recv_data[9];  // 从视觉上位机接收的数据-绝对角度，第9个字节作为识别到目标的标志位
static uint8_t vision_send_data[23]; // 给视觉上位机发送的数据-四元数

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

static Robot_Status_e robot_state; // 机器人整体工作状态

static referee_info_t *referee_data; // 用于获取裁判系统的数据

uint8_t UI_SendFlag = 1; // UI发送标志位

uint8_t auto_rune; // 自瞄打符标志位

float rec_yaw, rec_pitch;

uint8_t SuperCap_flag_from_user = 0; // 超电标志位

void HOST_RECV_CALLBACK()
{
    memcpy(vision_recv_data, host_instance->comm_instance, host_instance->RECV_SIZE);
    vision_recv_data[8] = 1;
}
void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个

    HostInstanceConf host_conf = {
        .callback  = HOST_RECV_CALLBACK,
        .comm_mode = HOST_VCP,
        .RECV_SIZE = 8,
    };
    host_instance = HostInit(&host_conf); // 视觉通信串口

    referee_data = RefereeHardwareInit(&huart6); // 裁判系统初始化,会同时初始化UI

    gimbal_cmd_pub  = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub   = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub  = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    ui_cmd_pub  = PubRegister("ui_cmd", sizeof(UI_Cmd_s));
    ui_feed_sub = SubRegister("ui_feed", sizeof(UI_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id      = 0x312,
            .rx_id      = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD

#if PITCH_FEED_TYPE
    gimbal_cmd_send.pitch = 0;
#else
    gimbal_cmd_send.pitch = PTICH_HORIZON_ANGLE;
#endif

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_data->referee_id.Robot_Color       = referee_data->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_data->referee_id.Cilent_ID         = 0x0100 + referee_data->GameRobotState.robot_id; // 计算客户端ID
    referee_data->referee_id.Robot_ID          = referee_data->GameRobotState.robot_id;          // 计算机器人ID
    referee_data->referee_id.Receiver_Robot_ID = 0;
}

float yaw_control;   // 遥控器YAW自由度输入值
float pitch_control; // 遥控器PITCH自由度输入值

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 * @todo  将单圈角度修改为-180~180
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    static float gimbal_yaw_current_angle;                                                // 云台yaw轴当前角度
    static float gimbal_yaw_set_angle;                                                    // 云台yaw轴目标角度
    angle                               = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
    gimbal_yaw_current_angle            = gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET];
    gimbal_yaw_set_angle                = yaw_control;
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
}

/**
 * @brief 对Pitch轴角度变化进行限位
 *
 */
static void PitchAngleLimit()
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
    if (pitch_control < limit_max)
        pitch_control = limit_max;
    if (pitch_control > limit_min)
        pitch_control = limit_min;
#endif

    // gimbal_cmd_send.pitch = pitch_control;
}

/**
 * @brief 云台Yaw轴反馈值改单圈角度后过圈处理
 *
 */
static void YawControlProcess()
{
    if (yaw_control - gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET] > 180) {
        yaw_control -= 360;
    } else if (yaw_control - gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET] < -180) {
        yaw_control += 360;
    }
}

static float heat_coef;

static void HeatControl()
{
    if (shoot_cmd_send.friction_mode == FRICTION_OFF) {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    static float rate_coef;
    if (heat_coef == 1)
        rate_coef = 1;
    else if (heat_coef >= 0.8 && heat_coef < 1)
        rate_coef = 0.8;
    else if (heat_coef >= 0.6 && heat_coef < 0.8)
        rate_coef = 0.6;
    else if (heat_coef < 0.6)
        rate_coef = 0.4;
    heat_coef = ((referee_data->GameRobotState.shooter_id1_17mm_cooling_limit - referee_data->PowerHeatData.shooter_17mm_heat0 + rate_coef * referee_data->GameRobotState.shooter_id1_17mm_cooling_rate) * 1.0f) / (1.0f * referee_data->GameRobotState.shooter_id1_17mm_cooling_limit);
    // 新热量管理
    if (referee_data->GameRobotState.shooter_id1_17mm_cooling_limit - 40 + 30 * heat_coef - shoot_fetch_data.shooter_local_heat <= shoot_fetch_data.shooter_heat_control) {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

// 底盘模式
uint8_t rc_mode[5];
#define CHASSIS_FREE     0
#define CHASSIS_ROTATION 1
#define CHASSIS_FOLLOW   2
#define SHOOT_FRICTION   3
#define SHOOT_LOAD       4

/**
 * @brief  紧急停止,双下
 *
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.friction_mode  = FRICTION_OFF;
    shoot_cmd_send.load_mode      = LOAD_STOP;
    shoot_cmd_send.shoot_mode     = SHOOT_OFF;
    SuperCap_flag_from_user       = SUPER_USER_CLOSE;
    memset(rc_mode, 1, sizeof(uint8_t));
    memset(rc_mode + 1, 0, sizeof(uint8_t) * 4);
    LOGERROR("[CMD] emergency stop!");
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    shoot_cmd_send.shoot_mode   = SHOOT_ON; // 发射机构常开
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    shoot_cmd_send.shoot_rate   = 30; // 射频默认30Hz

    if (rc_data[TEMP].rc.dial > 400) {
        SuperCap_flag_from_user = SUPER_USER_OPEN;
    } else {
        SuperCap_flag_from_user = SUPER_USER_CLOSE; // 默认关闭超电
    }

    // 使用相对角度控制
    memcpy(&rec_yaw, vision_recv_data, sizeof(float));
    memcpy(&rec_pitch, vision_recv_data + 4, sizeof(float));

    switch (rc_data[TEMP].rc.switch_right) {
        case RC_SW_MID:
            if (rc_mode[CHASSIS_FREE] == 1) {
                chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
            }

            if (rc_data[TEMP].rc.switch_left == RC_SW_MID && chassis_cmd_send.chassis_mode == CHASSIS_NO_FOLLOW) {
                rc_mode[CHASSIS_ROTATION] = 1;
                rc_mode[CHASSIS_FOLLOW]   = 1;
            }

            if (chassis_cmd_send.chassis_mode == CHASSIS_ROTATE) {
                rc_mode[CHASSIS_ROTATION] = 0;
            }

            if (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
                rc_mode[CHASSIS_FOLLOW] = 0;
            }
            break;
        case RC_SW_DOWN:
            rc_mode[CHASSIS_FREE] = 0;
            if (rc_mode[CHASSIS_ROTATE] == 1) {
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
            rc_mode[CHASSIS_FREE] = 0;
            if (rc_mode[CHASSIS_FOLLOW] == 1) {
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
                if (rc_mode[SHOOT_FRICTION] == 0 && shoot_cmd_send.friction_mode == FRICTION_OFF) {
                    rc_mode[SHOOT_FRICTION] = 1;
                }
            }
            if (shoot_cmd_send.friction_mode == FRICTION_ON) {
                rc_mode[SHOOT_FRICTION] = 0;
            }
            break;
        case RC_SW_UP:
            if (rc_mode[SHOOT_FRICTION] == 1) {
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

    HeatControl();

    // 底盘参数
    chassis_cmd_send.vx = 70.0f * (float)rc_data[TEMP].rc.rocker_r_; // 水平方向
    chassis_cmd_send.vy = 70.0f * (float)rc_data[TEMP].rc.rocker_r1; // 竖直方向

    yaw_control -= YAW_K * (float)rc_data[TEMP].rc.rocker_l_;
    pitch_control -= PITCH_K * (float)rc_data[TEMP].rc.rocker_l1;
    // 云台参数
    YawControlProcess();
    gimbal_cmd_send.yaw   = yaw_control;
    gimbal_cmd_send.pitch = pitch_control;
}

ramp_t fb_ramp;
ramp_t lr_ramp;
ramp_t slow_ramp;

/**
 * @brief 键盘设定速度
 *
 */
static void ChassisSet()
{
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;

    // 底盘移动
    static float current_speed_x = 0;
    static float current_speed_y = 0;
    // 前后移动
    // 防止逃跑时关小陀螺按Ctrl进入慢速模式
    if (rc_data[TEMP].key[KEY_PRESS].w) {
        chassis_cmd_send.vy = (current_speed_y + (CHASSIS_SPEED - current_speed_y) * ramp_calc(&fb_ramp)); // vx方向待测
        ramp_init(&slow_ramp, RAMP_TIME);                                                                  // 2000
    } else if (rc_data[TEMP].key[KEY_PRESS].s) {
        chassis_cmd_send.vy = (current_speed_y + (-CHASSIS_SPEED - current_speed_y) * ramp_calc(&fb_ramp));
        ramp_init(&slow_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w) { // 防止逃跑关小陀螺进入慢速移动
        chassis_cmd_send.vy = (current_speed_y + (4000 - current_speed_y) * ramp_calc(&slow_ramp));
        ramp_init(&fb_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s) {
        chassis_cmd_send.vy = (current_speed_y + (-4000 - current_speed_y) * ramp_calc(&slow_ramp));
        ramp_init(&fb_ramp, RAMP_TIME);
    } else {
        chassis_cmd_send.vy = 0;
        ramp_init(&fb_ramp, RAMP_TIME);
    }

    // 左右移动
    if (rc_data[TEMP].key[KEY_PRESS].a) {
        chassis_cmd_send.vx = (current_speed_x + (CHASSIS_SPEED - current_speed_x) * ramp_calc(&lr_ramp));
        ramp_init(&slow_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS].d) {
        chassis_cmd_send.vx = (current_speed_x + (-CHASSIS_SPEED - current_speed_x) * ramp_calc(&lr_ramp));
        ramp_init(&slow_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a) {
        chassis_cmd_send.vx = (current_speed_x + (+4000 - current_speed_x) * ramp_calc(&fb_ramp));
        ramp_init(&lr_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d) {
        chassis_cmd_send.vx = (current_speed_x + (-4000 - current_speed_x) * ramp_calc(&fb_ramp));
        ramp_init(&lr_ramp, RAMP_TIME);
    } else {
        chassis_cmd_send.vx = 0;
        ramp_init(&lr_ramp, RAMP_TIME);
    }

    current_speed_x = chassis_cmd_send.vx;
    current_speed_y = chassis_cmd_send.vy;
}

/**
 * @brief 鼠标移动云台
 *
 */
static void GimbalSet()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    // 相对角度控制
    memcpy(&rec_yaw, vision_recv_data, sizeof(float));
    memcpy(&rec_pitch, vision_recv_data + 4, sizeof(float));

    // 按住鼠标右键且视觉识别到目标
    yaw_control -= rc_data[TEMP].mouse.x / 350.0f;
    pitch_control -= -rc_data[TEMP].mouse.y / 15500.0f;

    YawControlProcess();
    gimbal_cmd_send.yaw   = yaw_control;
    gimbal_cmd_send.pitch = pitch_control;
}

/**
 * @brief 键鼠设定机器人发射模式
 *
 */
static void ShootSet()
{
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.shoot_rate = 30; // 射频默认30Hz

    // 仅在摩擦轮开启时有效
    if (shoot_cmd_send.friction_mode == FRICTION_ON) {
        // 打弹，单击左键单发，长按连发
        if (rc_data[TEMP].mouse.press_l) {
            // 打符，单发
            if (auto_rune == 1) {
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
    HeatControl();
}

/**
 * @brief 键盘处理模式标志位
 *
 */

static void KeyGetMode()
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
            if (UI_SendFlag == 1) {
                UI_SendFlag = 0;
            }
            break;
        case 0:
            UI_SendFlag = 1;
            break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].ctrl) {
        case 1:
            auto_rune = 1;
            break;
        case 0:
            auto_rune = 0;
            break;
        default:
            break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) {
        case 1:
            SuperCap_flag_from_user = SUPER_USER_OPEN;
            break;
        case 0:
            SuperCap_flag_from_user = SUPER_USER_CLOSE;
            break;
    }
}

/**
 * @brief 机器人复位函数，按下Ctrl+Shift+r
 *
 *
 */
static void RobotReset()
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
static void MouseKeySet()
{
    ChassisSet();
    GimbalSet();
    ShootSet();
    KeyGetMode();
    RobotReset(); // 机器人复位处理
}

extern referee_info_t *referee_data_for_ui;

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    DeterminRobotID();
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(ui_feed_sub, &ui_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_up(rc_data[TEMP].rc.switch_left) && (switch_is_down(rc_data[TEMP].rc.switch_right))) // 遥控器拨杆右[上]左[下],键鼠控制
        MouseKeySet();
    else if (RC_LOST || (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right))) {
        EmergencyHandler(); // 调试/疯车时急停
    } else {
        RemoteControlSet();
    }

    // 云台软件限位
    PitchAngleLimit(); // PITCH限位
    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
    // chassis
    memcpy(&chassis_cmd_send.chassis_power, &referee_data->PowerHeatData.chassis_power, sizeof(float));
    memcpy(&chassis_cmd_send.power_buffer, &referee_data->PowerHeatData.chassis_power_buffer, sizeof(uint16_t));
    memcpy(&chassis_cmd_send.level, &referee_data->GameRobotState.robot_level, sizeof(uint8_t));
    memcpy(&chassis_cmd_send.power_limit, &referee_data->GameRobotState.chassis_power_limit, sizeof(uint16_t));
    memcpy(&chassis_cmd_send.SuperCap_flag_from_user, &SuperCap_flag_from_user, sizeof(uint8_t));
    // memcpy

    // shoot
    memcpy(&shoot_cmd_send.shooter_heat_cooling_rate, &referee_data->GameRobotState.shooter_id1_17mm_cooling_rate, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.shooter_referee_heat, &referee_data->PowerHeatData.shooter_17mm_heat0, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.shooter_cooling_limit, &referee_data->GameRobotState.shooter_id1_17mm_cooling_limit, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.bullet_speed, &referee_data->ShootData.bullet_speed, sizeof(float));

    // UI
    // memcpy(referee_data_for_ui, referee_data, sizeof(referee_info_t));
    referee_data_for_ui = referee_data;
    memcpy(&ui_cmd_send.ui_send_flag, &UI_SendFlag, sizeof(uint8_t));
    memcpy(&ui_cmd_send.chassis_mode, &chassis_cmd_send.chassis_mode, sizeof(chassis_mode_e));
    memcpy(&ui_cmd_send.chassis_attitude_angle, &gimbal_fetch_data.yaw_motor_single_round_angle, sizeof(uint16_t));
    memcpy(&ui_cmd_send.friction_mode, &shoot_cmd_send.friction_mode, sizeof(friction_mode_e));
    memcpy(&ui_cmd_send.rune_mode, &auto_rune, sizeof(uint8_t));
    memcpy(&ui_cmd_send.SuperCap_mode, &chassis_fetch_data.CapFlag_open_from_real, sizeof(uint8_t));
    memcpy(&ui_cmd_send.SuperCap_voltage, &chassis_fetch_data.cap_voltage, sizeof(float));
    memcpy(&ui_cmd_send.Chassis_Ctrl_power, &chassis_fetch_data.chassis_power_output, sizeof(float));
    memcpy(&ui_cmd_send.Cap_absorb_power_limit, &chassis_fetch_data.capget_power_limit, sizeof(uint16_t));
    memcpy(&ui_cmd_send.Chassis_power_limit, &referee_data->GameRobotState.chassis_power_limit, sizeof(uint16_t));
    memcpy(&ui_cmd_send.Shooter_heat, &shoot_fetch_data.shooter_local_heat, sizeof(float));
    memcpy(&ui_cmd_send.Heat_Limit, &referee_data->GameRobotState.shooter_id1_17mm_cooling_limit, sizeof(uint16_t));

#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(ui_cmd_pub, (void *)&ui_cmd_send);

    // master
    static uint8_t frame_head[] = {0xAF, 0x32, 0x00, 0x12};
    memcpy(vision_send_data, frame_head, 4);
    memcpy(vision_send_data + 4, gimbal_fetch_data.gimbal_imu_data->INS_data.INS_quat, sizeof(float) * 4);
    memcpy(vision_send_data + 20, &referee_data->GameRobotState.robot_id, sizeof(uint8_t));
    memcpy(vision_send_data + 21, &auto_rune, sizeof(uint8_t));
    vision_send_data[22] = 0;
    for (size_t i = 0; i < 22; i++)
        vision_send_data[22] += vision_send_data[i];
    HostSend(host_instance, vision_send_data, 23);
}
