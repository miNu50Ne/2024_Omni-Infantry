#pragma once

#include <stdint.h>

#define CURRENT_2_TORQUE       (20.0f / 16384.0f)
#define TORQUE_COEFFICIENT     0.3f
#define CONVERSION_COEFFICIENT 9.55f
#define REDUCTION_RATIO_OF_DJI (187.0f / 3591.0f)

/**输入功率组件 */
typedef struct {
    float machine_power;
    float current_power;
    float speed_power;
    float static_consumption;
    float input_power;
    float total_power;
} Power_Input_t;

typedef struct {
    // 模型参数
    float current_coef;
    float velocity_coef;
    float torque_current_coefficient;
    float give_power;
    float power_scale;

    // 输入功率
    Power_Input_t input_power_components;

    // 更新值
    float reduction_ratio;
    uint16_t max_power;

    float torque_output;
} PowerCalcInstance;

// /**计算量 */
typedef struct
{
    float cmd_current[4];
    float wheel_velocity[4];

    uint8_t count;
} Power_Data_s;

// void PowerCalcInit(float reduction_ratio_init);

void maxpowerupdate(uint16_t max_power_init);

// float PowerInputCalc(float motor_speed, float motor_current);

float CurrentOutputCalc(float motor_power, float motor_speed, float motor_current);