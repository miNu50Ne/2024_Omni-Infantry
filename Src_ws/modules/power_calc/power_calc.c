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

static void limit_output(float *current_output, float limit)
{
    if (*current_output > limit) {
        *current_output = limit;
    }
    if (*current_output < -limit) {
        *current_output = -limit;
    }
}

// void PowerCalcInit(float reduction_ratio_init)
// {
//     powercalcinstance.current_coef  = 8.40e-08f;
//     powercalcinstance.velocity_coef = 4.10e-07f;

//     (reduction_ratio_init != 0) ? (powercalcinstance.reduction_ratio = reduction_ratio_init)
//                                 : (powercalcinstance.reduction_ratio = (REDUCTION_RATIO_OF_DJI));

//     powercalcinstance.torque_current_coefficient                = CURRENT_2_TORQUE * TORQUE_COEFFICIENT * powercalcinstance.reduction_ratio / CONVERSION_COEFFICIENT;
//     powercalcinstance.input_power_components.static_consumption = 1.0f;
// }

void maxpowerupdate(uint16_t max_power_init)
{
    powercalcinstance.max_power = max_power_init;
}

float CurrentOutputCalc(Power_Data_s *power_data)
{
    powercalcinstance.input_power_components.total_power = 0;
    for (size_t i = 0; i < 4; i++) {

        if (power_data->input_power[i] < 0) {
            continue;
        } else {
            powercalcinstance.input_power_components.total_power += power_data->input_power[i];
        }
    }
    return powercalcinstance.input_power_components.total_power;
}
