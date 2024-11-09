#pragma once

#include <stdint.h>

#define CURRENT_2_TORQUE       (20.0f / 16384)
#define TORQUE_COEFFICIENT     0.3
#define CONVERSION_COEFFICIENT 9.55
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
    float k1;
    float k2;
    float torque_current_coefficient;
    float give_power;
    float power_scale;

    Power_Input_t input_power_components;

    float reduction_ratio;
    uint16_t max_power;

    float torque_output;
} PowerCalcInstance;

/**计算量 */
typedef struct
{
    float input_power[4];
    float wheel_speed[4];
    float total_power;
    float predict_output[4];
    uint8_t count;
} Power_Data_s;

void PowerControlupdate(uint16_t max_power_init, float reduction_ratio_init);

float PowerInputCalc(float motor_speed, float motor_current);

float TotalPowerCalc(float input_power[]);

float CurrentOutputCalc(float motor_power, float motor_speed, float motor_current);