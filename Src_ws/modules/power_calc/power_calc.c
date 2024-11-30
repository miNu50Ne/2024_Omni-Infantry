/**
 * @file power_calc.c
 * @author miNu50Ne (minu50ne@qq.com)
 * @brief Refer to DynamicX open source solution
 * @version 0.1
 * @date 2024-11-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "power_calc.h"
#include "stdlib.h"

#include <stdlib.h>
#include <math.h>

static PowerCalcInstance powercalcinstance;

static float limit_output(float *val, float min, float max)
{
    if (*val > max) {
        *val = max;
    } else if (*val < min) {
        *val = min;
    }
    return *val;
}

void power_calc_params_init(float reduction_ratio_init, bool output_direction_init)
{
    powercalcinstance.current_coef  = 8.40e-08f;
    powercalcinstance.velocity_coef = 4.10e-07f;

    powercalcinstance.output_direction = output_direction_init;

    powercalcinstance.reduction_ratio = reduction_ratio_init != 0 ? reduction_ratio_init
                                                                  : REDUCTION_RATIO_OF_DJI;

    powercalcinstance.torque_current_coefficient = CMD_2_CURRENT * TORQUE_COEFFICIENT * powercalcinstance.reduction_ratio;
    powercalcinstance.static_consumption         = 1.0f;
}

void max_power_update(uint16_t max_power_init)
{
    powercalcinstance.max_power = max_power_init;
}

// p=t*w(b)+k1*w2(c)+k2*t2(a)
float current_output_calc(Power_Data_s *motors_data)
{
    float a = 0, b = 0, c = 0;
    for (size_t motor_id = 0; motor_id < 4; motor_id++) {
        motors_data->cmd_torque[motor_id] = motors_data->cmd_current[motor_id] * powercalcinstance.torque_current_coefficient;
        a += powercalcinstance.current_coef * powf(motors_data->cmd_torque[motor_id], 2.);
        b += motors_data->cmd_torque[motor_id] * motors_data->wheel_velocity[motor_id];
        c += powercalcinstance.velocity_coef * powf(motors_data->wheel_velocity[motor_id], 2.) - powercalcinstance.static_consumption;
    }

    powercalcinstance.zoom_coef = a + b + c < powercalcinstance.max_power ? 1.0 : (b * b - 4 * a * c) > 0 ? (-b + sqrtf(b * b - 4 * a * c)) / (2 * a)
                                                                                                          : 0.;

    return limit_output(&powercalcinstance.zoom_coef, 0.0, 1.0);
}
