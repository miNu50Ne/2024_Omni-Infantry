#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "tool.h"
#include "referee_UI.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
static uint32_t shoot_heat_count[2];

uint32_t shoot_count = 0;
// float d_watch;  // 创建一个全局变量来记录微分值，便于调试

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 5,
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut        = 20000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type    = SPEED_LOOP,
            .close_loop_type    = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id                             = 3; // 左摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;

    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id                             = 4; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r                                                        = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1, // 10
                .Ki            = 0, // 1
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut        = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type    = SPEED_LOOP,             // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type    = SPEED_LOOP,             //| ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_cmd_recv.shoot_mode = SHOOT_ON; // 初始化后摩擦轮进入准备模式,也可将右拨杆拨至上一次来手动开启

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

float Block_Time;        // 堵转时间
float Reverse_Time;      // 反转时间
float current_record[6]; // 第五个为最近的射速 第六个为平均射速
float Block_Status;      // 拨弹盘状态
/**
 * @brief 堵转，弹速检测
 *
 */
static void Load_Reverse()
{
    // 获取拨弹盘转速
    current_record[0] = current_record[1];
    current_record[1] = current_record[2];
    current_record[2] = current_record[3];
    current_record[3] = current_record[4];
    current_record[4] = loader->measure.real_current; // 第五个为最近的拨弹盘電流
    current_record[5] = (current_record[0] + current_record[1] + current_record[2] + current_record[3] + current_record[4]) / 5.0;

    if (current_record[5] > 10000) {
        Block_Time++;
    }

    // 反转
    if (Reverse_Time >= 1) {
        shoot_cmd_recv.load_mode = LOAD_REVERSE;
        Reverse_Time++;

        // 反转时间
        if (Reverse_Time >= 300) {
            Reverse_Time = 0;
            Block_Time   = 0;
        }
    }

    // 电流较大恢复正转
    if (loader->measure.real_current < -2000) {
        Reverse_Time = 0;
        Block_Time   = 0;
    }

    else {
        // 堵转时间5*发射任务周期（5ms）= 25ms
        if (Block_Time > 1) {
            Reverse_Time = 1;
        }
    }
}

int one_bullet;
/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    } else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        return;

    Load_Reverse();
    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode) {
        // 停止拨盘
        case LOAD_STOP:
            DJIMotorSetRef(loader, 0); // 同时设定参考值为0,这样停止的速度最快
            shoot_heat_count[0] = shoot_count;
            shoot_heat_count[1] = shoot_heat_count[0];
            one_bullet          = 0;
            break;
        // 激活能量机关
        case LOAD_1_BULLET:
            hibernate_time      = DWT_GetTimeline_ms(); // 记录触发指令的时间
            dead_time           = 150;
            shoot_heat_count[1] = shoot_count;
            if (shoot_heat_count[1] - shoot_heat_count[0] >= 1) {
                one_bullet = 1;
            }
            switch (one_bullet) {
                case 1:
                    DJIMotorSetRef(loader, 0);
                    break;
                case 0:
                    DJIMotorSetRef(loader, 10000);
                    break;
            }
            break;
        // 连发模式
        case LOAD_BURSTFIRE:
            DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
            // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
            break;
        case LOAD_REVERSE:
            DJIMotorSetRef(loader, -20000);
            // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
            break;
        default:
            while (1); // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        DJIMotorSetRef(friction_l, 40000);
        DJIMotorSetRef(friction_r, 40000);
    } else if (shoot_cmd_recv.friction_mode == FRICTION_REVERSE) {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    } else if (shoot_cmd_recv.friction_mode == FRICTION_OFF) // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

// 热量控制算法
void Shoot_Fric_data_process(void)
{
    /*----------------------------------变量常量------------------------------------------*/
    static bool bullet_waiting_confirm = false;                                         // 等待比较器确认
    // uint8_t shoot_speed                = referee_info.PowerHeatData.shooter_17mm_heat0; // 获取弹速 
    float data                         = friction_l->measure.speed_aps;                 // 获取摩擦轮转速
    static uint16_t data_histroy[MAX_HISTROY];                                          // 做循环队列
    static uint8_t head = 0, rear = 0;                                                  // 队列下标
    float moving_average[2];                                                            // 移动平均滤波
    uint8_t data_num;                                                                   // 循环队列元素个数
    float derivative;                                                                   // 微分
    /*-----------------------------------逻辑控制-----------------------------------------*/
    data = abs(data);
    /*入队*/
    data_histroy[head] = data;
    head++;
    head %= MAX_HISTROY;
    /*判断队列数据量*/
    data_num = (head - rear + MAX_HISTROY) % MAX_HISTROY;
    if (data_num >= Fliter_windowSize + 1) // 队列数据量满足要求
    {
        moving_average[0] = 0;
        moving_average[1] = 0;
        /*同时计算两个滤波*/
        for (uint8_t i = rear, j = rear + 1, index = rear; index < rear + Fliter_windowSize; i++, j++, index++) {
            i %= MAX_HISTROY;
            j %= MAX_HISTROY;
            moving_average[0] += data_histroy[i];
            moving_average[1] += data_histroy[j];
        }
        moving_average[0] /= Fliter_windowSize;
        moving_average[1] /= Fliter_windowSize;
        /*滤波求导*/
        derivative = moving_average[1] - moving_average[0];
        /*导数比较*/
        // d_watch = derivative;
        if (derivative < -300) {
            bullet_waiting_confirm = true;
        } else if (derivative > -100) {
            if (bullet_waiting_confirm == true) {
                local_heat += One_bullet_heat; // 确认打出
                shoot_count++;
                bullet_waiting_confirm = false;
            }
        }
        rear++;
        rear %= MAX_HISTROY;
    }
}