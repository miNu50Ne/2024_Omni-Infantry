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
#include <stdint.h>
#include <stdlib.h>
#include <math.h>



// static PowerCalcInstance powercalcinstance;

// static void limit_output(float *current_output, float limit)
// {
//     if (*current_output > limit) {
//         *current_output = limit;
//     }
//     if (*current_output < -limit) {
//         *current_output = -limit;
//     }
// }

// void PowerCalcInit(float reduction_ratio_init)
// {
//     powercalcinstance.current_coef  = 8.40e-08f;
//     powercalcinstance.velocity_coef = 4.10e-07f;

//     (reduction_ratio_init != 0) ? (powercalcinstance.reduction_ratio = reduction_ratio_init)
//                                 : (powercalcinstance.reduction_ratio = (REDUCTION_RATIO_OF_DJI));

//     powercalcinstance.torque_current_coefficient                = (20.0f / 16384.0f) * 0.3f * powercalcinstance.reduction_ratio / CONVERSION_COEFFICIENT;
//     powercalcinstance.input_power_components.static_consumption = 1.0f;
// }

// void PowerControlupdate(uint16_t max_power_init)
// {
//     powercalcinstance.max_power = max_power_init;
// }

// float PowerInputCalc(float motor_speed, float motor_current)
// {
//     powercalcinstance.input_power_components.machine_power = motor_current * powercalcinstance.torque_current_coefficient * motor_speed;
//     powercalcinstance.input_power_components.current_power = powercalcinstance.current_coef * motor_current * motor_current;
//     powercalcinstance.input_power_components.speed_power   = powercalcinstance.velocity_coef * motor_speed * motor_speed;
//     powercalcinstance.input_power_components.input_power =
//         powercalcinstance.input_power_components.machine_power +
//         powercalcinstance.input_power_components.speed_power +
//         powercalcinstance.input_power_components.current_power +
//         powercalcinstance.input_power_components.static_consumption;
//     return powercalcinstance.input_power_components.input_power;
// }

// float TotalPowerCalc(Power_Data_s *power_data)
// {
//     powercalcinstance.input_power_components.total_power = 0;
//     for (size_t i = 0; i < 4; i++) {

//         if (power_data->input_power[i] < 0) {
//             continue;
//         } else {
//             powercalcinstance.input_power_components.total_power += power_data->input_power[i];
//         }
//     }
//     return powercalcinstance.input_power_components.total_power;
// }

// uint8_t flag;
// float CurrentOutputCalc(float motor_power, float motor_speed, float motor_current)
// {
//     // 计算值超功率，缩放
//     if (powercalcinstance.input_power_components.total_power > powercalcinstance.max_power) {
//         powercalcinstance.power_scale = powercalcinstance.max_power / powercalcinstance.input_power_components.total_power;
//         powercalcinstance.give_power  = motor_power * powercalcinstance.power_scale;

//         flag = 1;
//         // 计算电流值
//         float a = powercalcinstance.current_coef;
//         float b = motor_speed * powercalcinstance.torque_current_coefficient;
//         float c = powercalcinstance.input_power_components.speed_power -
//                   powercalcinstance.give_power +
//                   powercalcinstance.input_power_components.static_consumption;
//         (motor_current > 0) ? (powercalcinstance.torque_output = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a))
//                             : (powercalcinstance.torque_output = (-b - sqrtf(b * b - 4 * a * c)) / (2 * a));
//         // 限幅
//         limit_output(&powercalcinstance.torque_output, 12000);
//         return powercalcinstance.torque_output;
//     }
//     flag = 0;
//     // 未超功率
//     limit_output(&motor_current, 12000);
//     return motor_current;
// }