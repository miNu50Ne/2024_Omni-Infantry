/**
 * @file power_calc.c
 * @author miNu50Ne (minu50ne@qq.com)
 * @brief Refer to Gmaster open source solution
 * @version 0.1
 * @date 2024-11-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "power_calc.h"
#include <stdlib.h>
#include <math.h>

static PowerCalcInstance powercalcinstance;

static void limit_output(float *current_output)
{
    if (*current_output > 15000) {
        *current_output = 15000;
    }
    if (*current_output < -15000) {
        *current_output = -15000;
    }
}

void PowerCalcInit()
{
    powercalcinstance.k1                                        = 2.60e-07f;
    powercalcinstance.k2                                        = 2.93e-08f;
    powercalcinstance.torque_current_coefficient                = CURRENT_2_TORQUE * TORQUE_COEFFICIENT * powercalcinstance.reduction_ratio / CONVERSION_COEFFICIENT;
    powercalcinstance.input_power_components.static_consumption = 1.0f;
}

void PowerControlupdate(uint16_t max_power_init, float reduction_ratio_init)
{
    powercalcinstance.max_power = max_power_init;
    (reduction_ratio_init != 0) ? (powercalcinstance.reduction_ratio = reduction_ratio_init)
                                : (powercalcinstance.reduction_ratio = (REDUCTION_RATIO_OF_DJI));
}

float PowerInputCalc(float motor_speed, float motor_current)
{
    powercalcinstance.input_power_components.machine_power = motor_current * powercalcinstance.torque_current_coefficient * motor_speed;
    powercalcinstance.input_power_components.current_power = powercalcinstance.k2 * motor_current * motor_current;
    powercalcinstance.input_power_components.speed_power   = powercalcinstance.k1 * motor_speed * motor_speed;
    powercalcinstance.input_power_components.input_power =
        powercalcinstance.input_power_components.machine_power +
        powercalcinstance.input_power_components.speed_power +
        powercalcinstance.input_power_components.current_power +
        powercalcinstance.input_power_components.static_consumption;
    return powercalcinstance.input_power_components.input_power;
}

float TotalPowerCalc(float input_power[])
{
    powercalcinstance.input_power_components.total_power = 0;
    for (int i = 0; i < 4; i++) {
        if (input_power[i] < 0) {
            continue;
        } else {
            powercalcinstance.input_power_components.total_power += input_power[i];
        }
    }
    return powercalcinstance.input_power_components.total_power;
}

float CurrentOutputCalc(float motor_power, float motor_speed, float motor_current)
{
    // 计算值超功率，缩放
    if (powercalcinstance.input_power_components.total_power > powercalcinstance.max_power) {
        powercalcinstance.power_scale = powercalcinstance.max_power / powercalcinstance.input_power_components.total_power;
        powercalcinstance.give_power  = motor_power * powercalcinstance.power_scale;
        // 负功率不缩放，直接返回
        // todo:也许可以删掉，在计算值超功率状态下只有个别电机做负功的情况大概率并不存在
        if (motor_power < 0) {
            limit_output(&motor_current);
            return motor_current;
        }
        // 计算电流值
        float a = powercalcinstance.k2;
        float b = motor_speed * powercalcinstance.torque_current_coefficient;
        float c = powercalcinstance.k1 * motor_speed * motor_speed - powercalcinstance.give_power + powercalcinstance.input_power_components.static_consumption;
        (motor_current > 0) ? (powercalcinstance.torque_output = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a))
                            : (powercalcinstance.torque_output = (-b - sqrtf(b * b - 4 * a * c)) / (2 * a));
        // 限幅
        limit_output(&powercalcinstance.torque_output);
        return powercalcinstance.torque_output;
    }
    // 未超功率
    limit_output(&motor_current);
    return motor_current;
}