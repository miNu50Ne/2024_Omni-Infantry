#pragma once

#include "ramp.h"
#include <stdint.h>

#define CMD_2_CURRENT          (20.0f / 16384.0f)
#define TORQUE_COEFFICIENT     0.3f
#define REDUCTION_RATIO_OF_DJI (187.0f / 3591.0f)

typedef struct {
    // 模型参数
    float current_coef;
    float velocity_coef;

    float torque_current_coefficient;
    float static_consumption;
    float reduction_ratio;
    bool output_direction;

    float zoom_coef;

    // 更新值
    uint16_t max_power;

    float torque_output;
} PowerCalcInstance;

// /**计算量 */
typedef struct
{
    float cmd_current[4];
    float cmd_torque[4];
    float wheel_velocity[4];
    uint8_t motor_id;
} Power_Data_s;

void power_calc_params_init(float reduction_ratio_init, bool output_direction_init);

void max_power_update(uint16_t max_power_init);

float current_output_calc(Power_Data_s *motors_data);