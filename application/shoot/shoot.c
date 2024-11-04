#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "ramp.h"
#include <stdint.h>

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机

static Publisher_t *shoot_pub;
static Subscriber_t *shoot_sub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;         // 来自cmd的发射控制信息
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1,
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
            .outer_loop_type    = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type    = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_cmd_recv.shoot_mode = SHOOT_ON; // 初始化后摩擦轮进入准备模式,也可将右拨杆拨至上一次来手动开启

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

    DJIMotorStop(friction_l);
    DJIMotorStop(friction_r);
    DJIMotorStop(loader);
}

float loader_velocity;         // 当前电机转速
float loader_current;          // 电机电流值
loader_status_e loader_status; // 拨弹盘状态
/**
 * @brief 堵转，弹速检测
 * @details 获取拨弹盘转速。
 * 根据转速判断拨弹盘工作状态：静止，正常工作，转动时突然卡弹，卡弹根本转不动。
 * 判断方式：电机转速与目标值对比
 * 拨弹盘回退：1-2颗弹丸
 */
void loader_status_update(void)
{
    static uint8_t loader_normal_count;   // 正常工作计时
    static uint8_t loader_jam_count = 50; // 卡弹计时
    static uint8_t loader_reverse_count;  // 反转计时
    static uint8_t loader_weakjam_count;  // 轻微卡弹计时
    // 获取拨弹盘转速
    loader_velocity = shoot_cmd_recv.shoot_rate;

    switch (loader_status) {
        case LOADER_IDLE:
            loader_normal_count  = 0;
            loader_weakjam_count = 0;
            loader_jam_count     = 0;
            loader_reverse_count = 0;

            if (loader_velocity > 25) {
                loader_status = LOADER_NORMAL;
            }
            break;
        case LOADER_NORMAL:
            loader_normal_count++;
            if (loader_normal_count > 40) { //
                if (loader_current < -2000) {
                    loader_status = LOADER_JAM;
                } else if (abs(loader_current) < 200) {
                    loader_status = LOADER_IDLE;
                }
            }
            break;
        case LOADER_JAM:
            shoot_cmd_recv.load_mode = LOAD_JAM;

            if (loader_current > -400) {
                loader_weakjam_count++;
                if (loader_weakjam_count > 100)
                    loader_status = LOADER_IDLE;
            } else {
                loader_jam_count--;
            }
            if (loader_jam_count == 0) {
                loader_status = LOADER_ROLLBACK;
            }
            break;
        case LOADER_ROLLBACK:
            shoot_cmd_recv.load_mode = LOAD_REVERSE;
            loader_reverse_count++;
            // 反转时间
            if (loader_reverse_count > 100) {
                loader_status = LOADER_IDLE;
            }
            break;
        default:
            loader_status = LOADER_IDLE;
            break;
    }
}

/**
 * @brief 拨弹盘电流均值滤波
 *
 *
 */
static float loader_cunrrent_mean_filter(void)
{
    static float current_data[MAX_HISTROY] = {0};           // 初始化为0
    static uint8_t head = 0, rear = 0;                      // 队列下标
    static float window_sum = 0;                            // 窗口中所有元素的和
    uint8_t data_count      = 0;                            // 数据量
    float motor_current     = loader->measure.real_current; // 获取拨弹盘电流
    float filter_current    = 0;                            // 滤波后的电流

    current_data[head] = motor_current;
    head               = (head + 1) % MAX_HISTROY;
    data_count         = (head - rear) % MAX_HISTROY;
    if (data_count >= Fliter_windowSize) {
        window_sum = 0;
        for (uint8_t i = rear, index = 0; index < Fliter_windowSize; i++, index++) {
            i %= MAX_HISTROY;
            window_sum += current_data[i];
        }
        filter_current = window_sum / Fliter_windowSize;
        rear++;
        rear %= MAX_HISTROY;
    }

    return filter_current;
}

int heat_control    = 25; // 热量控制
float local_heat    = 0;  // 本地热量
int One_bullet_heat = 10; // 打一发消耗热量
uint32_t shoot_count;     // 已发弹量
// 热量控制算法
static void shoot_Fric_data_process(void)
{
    /*----------------------------------变量常量------------------------------------------*/
    static bool bullet_waiting_confirm = false;                         // 等待比较器确认
    float data                         = friction_l->measure.speed_aps; // 获取摩擦轮转速
    static uint16_t data_histroy[MAX_HISTROY];                          // 做循环队列
    static uint8_t head = 0, rear = 0;                                  // 队列下标
    float moving_average[2];                                            // 移动平均滤波
    uint8_t data_num;                                                   // 循环队列元素个数
    float derivative;                                                   // 微分
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
        if (derivative < -300) {
            bullet_waiting_confirm = true;
        } else if (derivative > 30) {
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
/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    uint8_t one_bullet = 0;
    ramp_t fric_on_ramp, fric_off_ramp;
    float fric_speed = 0; // 摩擦轮转速参考值
    uint32_t shoot_heat_count[2];
    float shoot_speed = 0;
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
    shoot_cmd_recv.shoot_rate = 30;
    loader_status_update();
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
                    DJIMotorSetRef(loader, 5000);
                    break;
            }
            break;
        // 连发模式
        case LOAD_BURSTFIRE:
            DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE);
            // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
            break;
        case LOAD_JAM:
            DJIMotorSetRef(loader, -shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE);
            break;
        case LOAD_REVERSE:
            DJIMotorSetRef(loader, -40000);
            // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
            break;
        default:
            while (1); // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        fric_speed = (shoot_speed + (35500 - shoot_speed) * ramp_calc(&fric_on_ramp));
        ramp_init(&fric_off_ramp, 300);
    } else if (shoot_cmd_recv.friction_mode == FRICTION_OFF) {
        fric_speed = (shoot_speed + (0 - shoot_speed) * ramp_calc(&fric_off_ramp));
        ramp_init(&fric_on_ramp, 300);
    }

    DJIMotorSetRef(friction_l, fric_speed);
    DJIMotorSetRef(friction_r, fric_speed);
    shoot_speed = fric_speed;

    // 反馈数据
    memcpy(&shoot_feedback_data.shooter_local_heat, &local_heat, sizeof(float));
    memcpy(&shoot_feedback_data.shooter_heat_control, &heat_control, sizeof(int));

    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM14) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */
    if (htim->Instance == TIM6) {
        /*-------------------------------------------热量控制部分---------------------------------------------*/
        local_heat -= (shoot_cmd_recv.shooter_heat_cooling_rate / 1000.0f); // 1000Hz冷却
        if (local_heat < 0) {
            local_heat = 0;
        }
        if (shoot_cmd_recv.shooter_referee_heat - shoot_cmd_recv.shooter_cooling_limit >= 15) // 裁判系统判断已经超了热量
        {
            local_heat = shoot_cmd_recv.shooter_referee_heat;
        }
        shoot_Fric_data_process();
        loader_current = loader_cunrrent_mean_filter();
    }
    /* USER CODE END Callback 1 */
}