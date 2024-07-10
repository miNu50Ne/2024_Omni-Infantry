/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_init.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "rm_referee.h"
#include "arm_math.h"
<<<<<<< HEAD
#include "power_calc.h"
#include "tool.h"
=======
#include "tool.h"
#include "power_calc.h"
>>>>>>> master

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE     (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH    (TRACK_WIDTH / 2.0f)    // 半轮距
#define PERIMETER_WHEEL     (RADIUS_WHEEL * 2 * PI) // 轮子周长

#define LF                  0
#define RF                  1
#define RB                  2
#define LB                  3

#define SuperCap_PowerLimit 1200

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

SuperCapInstance *cap;                                              // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

// 为了方便调试加入的量
static uint8_t center_gimbal_offset_x = CENTER_GIMBAL_OFFSET_X; // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
static uint8_t center_gimbal_offset_y = CENTER_GIMBAL_OFFSET_Y; // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0

<<<<<<< HEAD
// 跟随模式底盘的pid
// 目前没有设置单位，有些不规范，之后有需要再改
static PIDInstance Chassis_Follow_PID = {
    .Kp            = 75,
    .Ki            = 0,
    .Kd            = 1.0,
    .DeadBand      = 6.0, // 跟随模式设置了死区，防止抖动
=======
extern uint8_t Super_flag;         // 超电的标志位
extern uint8_t Super_condition;    // 超电的开关状态
extern float Super_condition_volt; // 超电的电压

extern Power_Data_s power_data;
extern ramp_t super_ramp;
extern float total_power;

// 跟随模式底盘的pid
// 目前没有设置单位，有些不规范，之后有需要再改
static PIDInstance FollowMode_PID = {
    .Kp            = 40,  // 25,//25, // 50,//70, // 4.5
    .Ki            = 0,   // 0
    .Kd            = 1.0, // 0.0,  // 0.07,  // 0
    .DeadBand      = 0,   // 0.75,  //跟随模式设置了死区，防止抖动
>>>>>>> master
    .IntegralLimit = 3000,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut        = 20000,
};

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

float aim_power = 0;

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
<<<<<<< HEAD
                .Kp            = 1.0, // 4.5
=======
                .Kp            = 1.1, // kp太大会导致功率预测寄完
>>>>>>> master
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 15000,
            }},
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id                             = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);
<<<<<<< HEAD
=======

    referee_data = RefereeHardwareInit(&huart6); // 裁判系统初始化,会同时初始化UI
>>>>>>> master

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id      = 0X427, // 超级电容默认接收id
            .rx_id      = 0x300, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD

    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id      = 0x311,
            .rx_id      = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + center_gimbal_offset_x + HALF_WHEEL_BASE - center_gimbal_offset_y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - center_gimbal_offset_x + HALF_WHEEL_BASE - center_gimbal_offset_y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + center_gimbal_offset_x + HALF_WHEEL_BASE + center_gimbal_offset_y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - center_gimbal_offset_x + HALF_WHEEL_BASE + center_gimbal_offset_y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf = chassis_vx + chassis_vy + chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = -chassis_vx + chassis_vy + chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
}

<<<<<<< HEAD
static ramp_t super_ramp, limit_ramp;
static float Plimit, Power_Output, power_output;
=======
float Power_Buffer, Plimit, power_lecel;
// uint16_t power_limit;

>>>>>>> master
/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 * @param
 * @param
 *
 */
<<<<<<< HEAD
static void LimitChassisOutput()
{

    if (chassis_cmd_recv.power_buffer < 50 && chassis_cmd_recv.power_buffer >= 40)
        Plimit = 0.9 + (chassis_cmd_recv.power_buffer - 40) * 0.01; // 15
    else if (chassis_cmd_recv.power_buffer < 40 && chassis_cmd_recv.power_buffer >= 35)
        Plimit = 0.75 + (chassis_cmd_recv.power_buffer - 35) * (0.15f / 5);
    else if (chassis_cmd_recv.power_buffer < 35 && chassis_cmd_recv.power_buffer >= 30)
        Plimit = 0.6 + (chassis_cmd_recv.power_buffer - 30) * (0.15 / 5);
    else if (chassis_cmd_recv.power_buffer < 30 && chassis_cmd_recv.power_buffer >= 20)
        Plimit = 0.35 + (chassis_cmd_recv.power_buffer - 20) * (0.25f / 10);
    else if (chassis_cmd_recv.power_buffer < 20 && chassis_cmd_recv.power_buffer >= 10)
        Plimit = 0.15 + (chassis_cmd_recv.power_buffer - 10) * 0.01;
    else if (chassis_cmd_recv.power_buffer < 10 && chassis_cmd_recv.power_buffer > 0)
        Plimit = 0.05 + chassis_cmd_recv.power_buffer * 0.01;
    else if (chassis_cmd_recv.power_buffer == 60)
        Plimit = 1;

    Power_Output = (power_output + (chassis_cmd_recv.power_limit - power_output) * ramp_calc(&limit_ramp));
    PowerControlupdate(Power_Output - 15 + 20 * Plimit, 1.0f / REDUCTION_RATIO_WHEEL);
    power_output = Power_Output;

    ramp_init(&super_ramp, 300);
=======
float lf_limit, rf_limit, lb_limit, rb_limit;
// static float Power_Max = 60.0f;

float lf_power, lb_power, rf_power, rb_power;
// float vt_lf_Now, vt_rf_Now, vt_lb_Now, vt_rb_Now;

float speed_sample[4], current_sample[4];
float power_signal;
float chassis_power;

static void LimitChassisOutput()
{
    // 省赛功率控制
    Power_Buffer = referee_data->PowerHeatData.chassis_power_buffer;
    // if (referee_data->PowerHeatData.chassis_power_buffer < 50 && referee_data->PowerHeatData.chassis_power_buffer >= 40)
    //     Plimit = 0.9 + (referee_data->PowerHeatData.chassis_power_buffer - 40) * 0.01; // 15
    // else if (referee_data->PowerHeatData.chassis_power_buffer < 40 && referee_data->PowerHeatData.chassis_power_buffer >= 35)
    //     Plimit = 0.75 + (referee_data->PowerHeatData.chassis_power_buffer - 35) * (0.15f / 5);
    // else if (referee_data->PowerHeatData.chassis_power_buffer < 35 && referee_data->PowerHeatData.chassis_power_buffer >= 30)
    //     Plimit = 0.6 + (referee_data->PowerHeatData.chassis_power_buffer - 30) * (0.15 / 5);
    // else if (referee_data->PowerHeatData.chassis_power_buffer < 30 && referee_data->PowerHeatData.chassis_power_buffer >= 20)
    //     Plimit = 0.35 + (referee_data->PowerHeatData.chassis_power_buffer - 20) * (0.25f / 10);
    // else if (referee_data->PowerHeatData.chassis_power_buffer < 20 && referee_data->PowerHeatData.chassis_power_buffer >= 10)
    //     Plimit = 0.15 + (referee_data->PowerHeatData.chassis_power_buffer - 10) * 0.01;
    // else if (referee_data->PowerHeatData.chassis_power_buffer < 10 && referee_data->PowerHeatData.chassis_power_buffer > 0)
    //     Plimit = 0.05 + referee_data->PowerHeatData.chassis_power_buffer * 0.01;
    // else if (referee_data->PowerHeatData.chassis_power_buffer == 60)
    //     Plimit = 1;

    power_lecel = referee_data->GameRobotState.robot_level * 0.1 + 0.8 + 0.15;

    // vt_lf = vt_lf * Plimit * power_lecel;
    // vt_rf = vt_rf * Plimit * power_lecel;
    // vt_lb = vt_lb * Plimit * power_lecel;
    // vt_rb = vt_rb * Plimit * power_lecel;

    PowerControlInit(referee_info.GameRobotState.chassis_power_limit + referee_data->PowerHeatData.chassis_power_buffer * 0.5 - 10, 1.0f / REDUCTION_RATIO_WHEEL); // 初始化功率控制
>>>>>>> master

    // 设定速度参考值
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

// 提高功率上限，飞坡或跑路
static void SuperLimitOutput()
{
<<<<<<< HEAD
    Power_Output = (power_output + (800 - power_output) * ramp_calc(&super_ramp));
    PowerControlupdate(Power_Output, 1.0f / REDUCTION_RATIO_WHEEL);

    power_output = Power_Output;

    ramp_init(&limit_ramp, 300);

=======
    // 飞坡速度，待测
    vt_lf *= 1.5;
    vt_rf *= 1.5;
    vt_lb *= 1.5;
    vt_rb *= 1.5;

    aim_power = (total_power + (SuperCap_PowerLimit - total_power) * ramp_calc(&super_ramp)); // vx方向待测
    PowerControlInit(SuperCap_PowerLimit, 1.0f / REDUCTION_RATIO_WHEEL);                      // 初始化功率控制
>>>>>>> master
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 超电控制算法
 *
 *
 */
<<<<<<< HEAD
uint8_t Super_Voltage_Allow_Flag;
int supercap_accel_delay, supercap_moderate_delay;

static SuperCap_State_e SuperCap_state = SUPER_STATE_LOW;
float voltage;
void Super_Cap_control()
{
    // 电容电压
    voltage = cap->cap_msg_s.CapVot;
=======
uint8_t UIflag = 1;
uint8_t Super_Allow_Flag;
uint8_t Super_Condition;
int super_time_delay;

static

SuperCap_State_e SuperCap_state = SUPER_STATE_LOW;

void Super_Cap_control()
{
    float voltage = cap->cap_msg_s.CapVot;
>>>>>>> master

    // 状态机逻辑
    switch (SuperCap_state) {
        case SUPER_STATE_LOW:
            if (voltage > SUPER_VOLTAGE_THRESHOLD_HIGH) {
                SuperCap_state = SUPER_STATE_HIGH;
            }
            break;
<<<<<<< HEAD
=======

>>>>>>> master
        case SUPER_STATE_HIGH:
            if (voltage < SUPER_VOLTAGE_THRESHOLD_LOW) {
                SuperCap_state = SUPER_STATE_LOW;
            }
            break;
<<<<<<< HEAD
        default:
            SuperCap_state = SUPER_STATE_LOW;
            break;
=======
>>>>>>> master
    }

    // 小于12V关闭
    if (SuperCap_state == SUPER_STATE_LOW) {
<<<<<<< HEAD
        Super_Voltage_Allow_Flag = SUPER_VOLTAGE_CLOSE;
    } else if (SuperCap_state == SUPER_STATE_HIGH) {
        Super_Voltage_Allow_Flag = SUPER_VOLTAGE_OPEN;
=======
        Super_Allow_Flag = SUPER_RELAY_CLOSE;
>>>>>>> master
    } else {
        // none
    }

    // User允许开启电容 且 电压充足
    if (Super_Voltage_Allow_Flag == SUPER_VOLTAGE_OPEN && chassis_cmd_recv.SuperCap_flag_from_user == SUPER_USER_OPEN) {
        cap->cap_msg_g.power_relay_flag = SUPER_RELAY_OPEN;
        supercap_moderate_delay         = 0;
    } else {
        LimitChassisOutput();
        supercap_moderate_delay++;
        if (supercap_moderate_delay > 100) {
            supercap_moderate_delay         = 101;
            cap->cap_msg_g.power_relay_flag = SUPER_RELAY_CLOSE;
        } else {
            cap->cap_msg_g.power_relay_flag = SUPER_RELAY_OPEN;
        }
    }

    // 物理层继电器状态改变，功率限制状态改变
<<<<<<< HEAD
    if (cap->cap_msg_s.SuperCap_open_flag_from_real == SUPERCAP_OPEN_FLAG_FROM_REAL_OPEN) {
        // 延时，超电打开后再提高功率
        supercap_accel_delay++;
        if (supercap_accel_delay > 30) {
            supercap_accel_delay = 31;
            SuperLimitOutput();
        } else {
            LimitChassisOutput();
        }
    } else {
        supercap_accel_delay = 0;
    }
=======
    if (cap->cap_msg_s.SuperCap_open_flag_from_real == SUPERCAP_OPEN_FLAG_FROM_REAL_Closed) {
        super_time_delay = 0;
        LimitChassisOutput();
        ramp_init(&super_ramp, SUPER_RAMP_TIME);
    } else {
        super_time_delay++;
        if (super_time_delay > 40) {
            super_time_delay = 41;
            No_Limit_Control();
        } else {
            LimitChassisOutput();
            ramp_init(&super_ramp, SUPER_RAMP_TIME);
        }
    }

>>>>>>> master
}

// 获取功率裆位
static void Power_level_get()
{
    switch (chassis_cmd_recv.level) {
        case 1:
            cap->cap_msg_g.power_level = 1;
            break;
        case 2:
            cap->cap_msg_g.power_level = 2;
            break;
        case 3:
            cap->cap_msg_g.power_level = 3;
            break;
        case 4:
            cap->cap_msg_g.power_level = 4;
            break;
        case 5:
            cap->cap_msg_g.power_level = 5;
            break;
        case 6:
            cap->cap_msg_g.power_level = 6;
            break;
        case 7:
            cap->cap_msg_g.power_level = 7;
            break;
        case 8:
            cap->cap_msg_g.power_level = 8;
            break;
        case 9:
            cap->cap_msg_g.power_level = 9;
            break;
        case 10:
            cap->cap_msg_g.power_level = 9;
            break;
        default:
            cap->cap_msg_g.power_level = 0;
            break;
    }
    if (chassis_cmd_recv.power_limit > robot_power_level_9to10) {
        cap->cap_msg_g.power_level = 9;
    }

<<<<<<< HEAD
    cap->cap_msg_g.chassic_power_remaining = chassis_cmd_recv.power_buffer;
}

static float chassis_vw, current_speed_vw, vw_set;
static ramp_t rotate_ramp;
=======
    cap->cap_msg_g.chassic_power_remaining = referee_data->PowerHeatData.chassis_power_buffer;
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...
}

>>>>>>> master
/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif                                                         // CHASSIS_BOARD
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    } else { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    float offset_angle;
    static float sin_theta, cos_theta;
    // 根据控制模式设定旋转速度
    static float sin_theta, cos_theta;
    switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_NO_FOLLOW:
            // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
            chassis_cmd_recv.wz = 0;

<<<<<<< HEAD
            cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            ramp_init(&rotate_ramp, 250);
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:                                                      // 跟随云台
            if (chassis_cmd_recv.offset_angle <= 90 && chassis_cmd_recv.offset_angle >= -90) // 0附近
                offset_angle = chassis_cmd_recv.offset_angle;
            else {
                offset_angle = chassis_cmd_recv.offset_angle >= 0 ? chassis_cmd_recv.offset_angle - 180 : chassis_cmd_recv.offset_angle + 180;
            }
            chassis_cmd_recv.wz = 1.2 * PIDCalculate(&Chassis_Follow_PID, offset_angle, 0);

            cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

            ramp_init(&rotate_ramp, 250);
            break;
        case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
            if (cap->cap_msg_s.SuperCap_open_flag_from_real == SUPERCAP_OPEN_FLAG_FROM_REAL_CLOSE) {
                vw_set = 5000;
            } else {
                vw_set = 7000;
            }
            chassis_vw       = (current_speed_vw + (vw_set - current_speed_vw) * ramp_calc(&rotate_ramp));
            current_speed_vw = chassis_vw;

            chassis_cmd_recv.wz = chassis_vw;
            cos_theta           = arm_cos_f32((chassis_cmd_recv.offset_angle + 22) * DEGREE_2_RAD); // 矫正小陀螺偏心
            sin_theta           = arm_sin_f32((chassis_cmd_recv.offset_angle + 22) * DEGREE_2_RAD);
            chassis_cmd_recv.vx *= 0.6;
            chassis_cmd_recv.vy *= 0.6;
=======
            cos_theta  = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta  = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
            chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台
            // chassis_cmd_recv.offset_angle += 360;                                            // 将角度映射到0-360度
            // if (chassis_cmd_recv.offset_angle <= 90 || chassis_cmd_recv.offset_angle >= 270) // 0附近
            //     offset_angle = chassis_cmd_recv.offset_angle <= 90 ? chassis_cmd_recv.offset_angle : (chassis_cmd_recv.offset_angle - 360);
            // else
            //     offset_angle = chassis_cmd_recv.offset_angle - 180;
            // chassis_cmd_recv.wz = -1.5f * offset_angle * abs(offset_angle);
            // chassis_cmd_recv.wz = chassis_cmd_recv.wz * 1.5;

            // chassis_cmd_recv.offset_angle += 360;                                            // 将角度映射到0-360度
            if (chassis_cmd_recv.offset_angle <= 90 || chassis_cmd_recv.offset_angle >= 270) // 0附近
                offset_angle = chassis_cmd_recv.offset_angle <= 90 ? chassis_cmd_recv.offset_angle : (chassis_cmd_recv.offset_angle - 360);
            else
                offset_angle = chassis_cmd_recv.offset_angle - 180;
            chassis_cmd_recv.wz = PIDCalculate(&FollowMode_PID, offset_angle, 0);
            chassis_cmd_recv.wz = chassis_cmd_recv.wz * 1.5;

            cos_theta  = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta  = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
            chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
            break;
        case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
            // chassis_cmd_recv.wz = 6000*(1-power_data.total_power/(referee_info.GameRobotState.chassis_power_limit + referee_data->PowerHeatData.chassis_power_buffer*1.2));
            chassis_cmd_recv.wz = 5000;

            cos_theta  = arm_cos_f32((chassis_cmd_recv.offset_angle + 20) * DEGREE_2_RAD); // 矫正小陀螺偏心
            sin_theta  = arm_sin_f32((chassis_cmd_recv.offset_angle + 20) * DEGREE_2_RAD);
            chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
            chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
>>>>>>> master
            break;
        case CHASSIS_REVERSE_ROTATE:
            chassis_cmd_recv.wz = -4000;
            cos_theta           = arm_cos_f32((chassis_cmd_recv.offset_angle + 22) * DEGREE_2_RAD); // 矫正小陀螺偏心
            sin_theta           = arm_sin_f32((chassis_cmd_recv.offset_angle + 22) * DEGREE_2_RAD);
        default:
            break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
<<<<<<< HEAD
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
=======
    // static float sin_theta, cos_theta;
    // cos_theta  = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    // sin_theta  = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    // chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    // chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
>>>>>>> master

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    Super_Cap_control();

<<<<<<< HEAD
=======
    // 根据电机的反馈速度和IMU(如果有)计算真实速度，根据超电的状态来输出功率，目前没有
    EstimateSpeed();

>>>>>>> master
    // 获得给电容传输的电容吸取功率等级
    Power_level_get();

    // 给电容传输数据
    SuperCapSend(cap, (uint8_t *)&cap->cap_msg_g);

    // 推送反馈消息
    memcpy(&chassis_feedback_data.CapFlag_open_from_real, &cap->cap_msg_s.SuperCap_open_flag_from_real, sizeof(uint8_t));
    memcpy(&chassis_feedback_data.cap_voltage, &cap->cap_msg_s.CapVot, sizeof(float));

#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}
