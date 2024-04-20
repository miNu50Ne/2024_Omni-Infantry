#include "power_control.h"
#include "arm_math.h"
#include "message_center.h"
#include "user_lib.h"
#include <math.h>
#include <stdlib.h>

#define MOTOR_LF 0
#define MOTOR_LB 1
#define MOTOR_RF 2
#define MOTOR_RB 3

float k1 = 1.23e-07;
float k2 = 1.453e-07;
// float constant = 4.081f;
float constant          = 0;
float toque_coefficient = 1.99688994e-6f;
float reduction_ratio, total_power;
float power_data[4];
float motor_current_output;
uint16_t max_power;
float power_scale;
float temp_positive, temp_negative;
float a, b, c;
float delta, delta_sqrt;
void PowerControlInit(uint16_t max_power_init, float reduction_ratio_init)
{
    max_power = max_power_init;
    if (reduction_ratio_init != 0) {
        reduction_ratio = reduction_ratio_init;
    } else {
        reduction_ratio = 1 / 13.0f; //(187.0f / 3591.0f);
    }
}
float PowerInputCalc(float motor_speed, float motor_current)
{
    float power_input = motor_current * toque_coefficient * motor_speed +
                        k2 * motor_speed * motor_speed +
                        k1 * motor_current * motor_current + constant;
    return power_input;
}
void TotalPowerCalc(float power_lf, float power_lb, float power_rf,
                    float power_rb)
{
    total_power   = 0;
    power_data[0] = power_lf;
    power_data[1] = power_lb;
    power_data[2] = power_rf;
    power_data[3] = power_rb;
    for (int i = 0; i < 4; i++) {
        if (power_data[i] < 0) {
            continue;
        } else {
            total_power += power_data[i];
        }
    }
}
float PowerCalc(float motor_power, float motor_speed, float motor_current)
{
    if (total_power > max_power) {
        power_scale = max_power / total_power;
        motor_power *= power_scale;
        if (motor_power < 0) {
            return motor_current;
        }
        a          = k1;
        b          = toque_coefficient * motor_speed;
        c          = k2 * motor_speed * motor_speed - motor_power + constant;
        delta      = b * b - 4 * a * c;
        delta_sqrt = sqrtf(b * b - 4 * a * c);
        if (motor_current > 0) {
            temp_positive        = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);
            motor_current_output = temp_positive;
        } else {
            temp_negative        = (-b - sqrtf(b * b - 4 * a * c)) / (2 * a);
            motor_current_output = temp_negative;
        }
        if (motor_current_output > 15000) {
            motor_current_output = 15000;
        }
        if (motor_current_output < -15000) {
            motor_current_output = -15000;
        }
        return motor_current_output;
    }
    return motor_current;
}