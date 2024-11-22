/**
 * @file chassis_controller.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 2.0
 * @date 2024-11-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "controller.h"
#include "robot_def.h"
#include "chassis_controller.h"
#include "power_controller.h"

#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"

#include "power_calc.h"
#include "general_def.h"
#include "arm_math.h"
#include "ramp.h"

static Publisher_t *chassis_pub;  // 用于发布底盘的数据
static Subscriber_t *chassis_sub; // 用于订阅底盘的控制命令

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

static ChassisInstance chassis_media_param; // 底盘中介变量

void ChassisDeviceInit()
{
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1.0,
                .Ki            = 0,
                .Kd            = 0,
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
    chassis_motor_config.can_init_config.tx_id                             = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id      = 0X427, // 超级电容默认接收id
            .rx_id      = 0x300, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    PowerCalcInit(); // 功率计算参数初始化
}

void ChassisParamInit()
{
    PID_Init_Config_s follow_pid = {
        .Kp            = 1.0,
        .Ki            = 0,
        .Kd            = 0,
        .IntegralLimit = 3000,
        .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut        = 20000,
        .DeadBand      = 10,
    };

    PIDInit(&chassis_media_param.chassis_follow_cotroller, &follow_pid);
    // 为了方便调试加入的量
    chassis_media_param.center_gimbal_offset_x = CENTER_GIMBAL_OFFSET_X; // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
    chassis_media_param.center_gimbal_offset_y = CENTER_GIMBAL_OFFSET_Y; // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0

    ramp_init(&chassis_media_param.rotate_ramp, 200);
}

void ChassisMsgInit()
{
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

void OmniCalculate()
{
    chassis_media_param.chassis_vx = chassis_cmd_recv.vx * chassis_media_param.cos_theta - chassis_cmd_recv.vy * chassis_media_param.sin_theta;
    chassis_media_param.chassis_vy = chassis_cmd_recv.vx * chassis_media_param.sin_theta + chassis_cmd_recv.vy * chassis_media_param.cos_theta;

    chassis_media_param.vt_lf = chassis_media_param.chassis_vx + chassis_media_param.chassis_vy + chassis_cmd_recv.wz * LF_CENTER;
    chassis_media_param.vt_rf = chassis_media_param.chassis_vx - chassis_media_param.chassis_vy + chassis_cmd_recv.wz * RF_CENTER;
    chassis_media_param.vt_rb = -chassis_media_param.chassis_vx - chassis_media_param.chassis_vy + chassis_cmd_recv.wz * RB_CENTER;
    chassis_media_param.vt_lb = -chassis_media_param.chassis_vx + chassis_media_param.chassis_vy + chassis_cmd_recv.wz * LB_CENTER;

    DJIMotorSetRef(motor_lf, chassis_media_param.vt_lf);
    DJIMotorSetRef(motor_rf, chassis_media_param.vt_rf);
    DJIMotorSetRef(motor_lb, chassis_media_param.vt_lb);
    DJIMotorSetRef(motor_rb, chassis_media_param.vt_rb);
}

void ChassisModeSet()
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    } else {
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    PowerController(cap, chassis_cmd_recv.power_buffer, chassis_cmd_recv.power_limit, chassis_cmd_recv.SuperCap_flag_from_user);

    switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_NO_FOLLOW:
            chassis_cmd_recv.wz = 0;

            chassis_media_param.cos_theta = arm_cos_f32(chassis_cmd_recv.gimbal_error_angle * DEGREE_2_RAD);
            chassis_media_param.sin_theta = arm_sin_f32(chassis_cmd_recv.gimbal_error_angle * DEGREE_2_RAD);
            ramp_init(&chassis_media_param.rotate_ramp, 250);
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:

            // if (chassis_cmd_recv.gimbal_error_angle <= 90 && chassis_cmd_recv.gimbal_error_angle >= -90) // 0附近
            //     gimbal_error_angle = chassis_cmd_recv.gimbal_error_angle;
            // else {
            //     gimbal_error_angle = chassis_cmd_recv.gimbal_error_angle >= 0 ? chassis_cmd_recv.gimbal_error_angle - 180 : chassis_cmd_recv.gimbal_error_angle + 180;
            // }

            chassis_media_param.chassis_vw = chassis_media_param.current_speed_vw +
                                             (PIDCalculate(&chassis_media_param.chassis_follow_cotroller, chassis_cmd_recv.gimbal_error_angle * 100, 0) -
                                              chassis_media_param.current_speed_vw) +
                                             ramp_calc(&chassis_media_param.rotate_ramp);
            chassis_media_param.current_speed_vw = chassis_media_param.chassis_vw;

            chassis_cmd_recv.wz = chassis_media_param.chassis_vw;

            chassis_media_param.cos_theta = arm_cos_f32(chassis_cmd_recv.gimbal_error_angle * DEGREE_2_RAD);
            chassis_media_param.sin_theta = arm_sin_f32(chassis_cmd_recv.gimbal_error_angle * DEGREE_2_RAD);

            break;
        case CHASSIS_ROTATE:
            chassis_cmd_recv.wz           = 5000;
            chassis_media_param.cos_theta = arm_cos_f32((chassis_cmd_recv.gimbal_error_angle + 22) * DEGREE_2_RAD); // 矫正小陀螺偏心
            chassis_media_param.sin_theta = arm_sin_f32((chassis_cmd_recv.gimbal_error_angle + 22) * DEGREE_2_RAD);
            chassis_cmd_recv.vx *= 0.6;
            chassis_cmd_recv.vy *= 0.6;
            break;
            ramp_init(&chassis_media_param.rotate_ramp, 250);

        case CHASSIS_REVERSE_ROTATE:
            chassis_cmd_recv.wz           = -5000;
            chassis_media_param.cos_theta = arm_cos_f32((chassis_cmd_recv.gimbal_error_angle + 22) * DEGREE_2_RAD); // 矫正小陀螺偏心
            chassis_media_param.sin_theta = arm_sin_f32((chassis_cmd_recv.gimbal_error_angle + 22) * DEGREE_2_RAD);
            chassis_cmd_recv.vx *= 0.6;
            chassis_cmd_recv.vy *= 0.6;
            ramp_init(&chassis_media_param.rotate_ramp, 250);
            break;
        default:
            break;
    }
}

void ChassisMsgComm()
{
    // 获取新的控制信息
    SubGetMessage(chassis_sub, &chassis_cmd_recv);

    // 给电容传输数据
    SuperCapSend(cap, (uint8_t *)&cap->cap_msg_g);

    // 推送反馈消息
    memcpy(&chassis_feedback_data.CapFlag_open_from_real, &cap->cap_msg_s.SuperCap_open_flag_from_real, sizeof(uint8_t));
    memcpy(&chassis_feedback_data.cap_voltage, &cap->cap_msg_s.CapVot, sizeof(float));

    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}
