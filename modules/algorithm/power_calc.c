#include "power_calc.h"
#include "arm_math.h"
#include "user_lib.h"
#include <stdlib.h>
#include <math.h>

float k1                = 5.636e-08f;
float k2                = -9.359e-09f;
float constant          = 0.5662f;
float toque_coefficient = 2.949745771e-06f; // (20/16384) * (0.3) / (9.55)

float reduction_ratio, total_power;
uint16_t max_power;

void PowerControlInit(uint16_t max_power_init, float reduction_ratio_init)
{
    int cnt   = 0;
    max_power = max_power_init;
    if (reduction_ratio_init != 0) {
        reduction_ratio = reduction_ratio_init;
    } else {
        reduction_ratio = (187.0f / 3591.0f);
    }

    // if (cnt == 0) {
    //     toque_coefficient *= reduction_ratio;
    //     cnt++;
    // }
}

float PowerInputCalc(float motor_speed, float motor_current)
{
    float power_input = motor_current * toque_coefficient * motor_speed +
                        k1 * motor_speed * motor_speed +
                        k2 * motor_current * motor_current + constant;
    return power_input;
}

float TotalPowerCalc(float input_power[])
{
    total_power = 0;
    for (int i = 0; i < 4; i++) {
        if (input_power[i] < 0) {
            continue;
        } else {
            total_power += input_power[i];
        }
    }
    return total_power;
}

float CurrentOutputCalc(float motor_power, float motor_speed, float motor_current)
{
    if (total_power > max_power) {
        float power_scale = max_power / total_power;
        motor_power *= power_scale ;
        if (motor_power < 0) {
            if (motor_current > 15000) {
                motor_current = 15000;
            }
            if (motor_current < -15000) {
                motor_current = -15000;
            }
            return motor_current;
        }
        float a = k1;
        float b = toque_coefficient * motor_speed;
        float c = k2 * motor_speed * motor_speed - motor_power + constant;
        if (motor_current > 0) {
            float temp    = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);
            motor_current = temp;
        } else {
            float temp    = (-b - sqrtf(b * b - 4 * a * c)) / (2 * a);
            motor_current = temp;
        }
        if (motor_current > 15000) {
            motor_current = 15000;
        }
        if (motor_current < -15000) {
            motor_current = -15000;
        }
        return motor_current;
    }
    if (motor_current > 15000) {
        motor_current = 15000;
    }
    if (motor_current < -15000) {
        motor_current = -15000;
    }
    return motor_current;
}