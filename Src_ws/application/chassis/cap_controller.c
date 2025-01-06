#include "super_cap.h"
#include "ramp.h"
#include "robot_def.h"
#include "power_calc.h"

static float power_set;

static void LimitChassisOutput(uint16_t power_buffer, uint16_t power_limit, ramp_t *s_ramp_)
{
    float Plimit = 0;

    // 缓冲能量闭环
    if (power_buffer < 50 && power_buffer >= 40)
        Plimit = 0.9 + (power_buffer - 40) * 0.01;
    else if (power_buffer < 40 && power_buffer >= 35)
        Plimit = 0.75 + (power_buffer - 35) * (0.15f / 5);
    else if (power_buffer < 35 && power_buffer >= 30)
        Plimit = 0.6 + (power_buffer - 30) * (0.15 / 5);
    else if (power_buffer < 30 && power_buffer >= 20)
        Plimit = 0.35 + (power_buffer - 20) * (0.25f / 10);
    else if (power_buffer < 20 && power_buffer >= 10)
        Plimit = 0.15 + (power_buffer - 10) * 0.01;
    else if (power_buffer < 10 && power_buffer > 0)
        Plimit = 0.05 + power_buffer * 0.01;
    else if (power_buffer == 60)
        Plimit = 1;

    power_set = 80;
    // Power_Output = power_limit - 10 + 20 * Plimit;
    max_power_update(power_set);

    ramp_init(s_ramp_, 300);
}

static void SuperLimitOutput(float cap_voltage, ramp_t *s_ramp_)
{
    static float power_output;
    power_set = (power_output + (250 - 20 + 40 * (cap_voltage - 17.0f) / 6.0f - power_output) * ramp_calc(s_ramp_));
    max_power_update(power_set);
    power_output = power_set;
}

/**
 * @brief 超电开关
 *
 *
 */
uint8_t Super_Voltage_Allow_Flag;

void cap_controller(SuperCapInstance *cap, uint16_t power_buffer, uint16_t power_limit, uint8_t switch_from_user)
{
    SuperCap_State_e SuperCap_state = SUPER_STATE_LOW;
    // 状态机逻辑,滞回
    switch (SuperCap_state) {
        case SUPER_STATE_LOW:
            if (cap->cap_msg_s.CapVot > SUPER_VOLTAGE_THRESHOLD_HIGH) {
                SuperCap_state = SUPER_STATE_HIGH;
            }
            Super_Voltage_Allow_Flag = SUPER_VOLTAGE_CLOSE;

            break;
        case SUPER_STATE_HIGH:
            if (cap->cap_msg_s.CapVot < SUPER_VOLTAGE_THRESHOLD_LOW) {
                SuperCap_state = SUPER_STATE_LOW;
            }
            Super_Voltage_Allow_Flag = SUPER_VOLTAGE_OPEN;

            break;
        default:
            SuperCap_state = SUPER_STATE_LOW;
            break;
    }

    ramp_t super_ramp;
    // User允许开启电容 且 电压充足
    switch (switch_from_user) {
        case SUPER_USER_OPEN:
            cap->cap_msg_g.enabled = SUPER_CMD_OPEN;
            SuperLimitOutput(cap->cap_msg_s.CapVot, &super_ramp);
            break;
        case SUPER_USER_CLOSE:
            cap->cap_msg_g.enabled = SUPER_CMD_CLOSE;
            LimitChassisOutput(power_buffer, power_limit, &super_ramp);
            break;
    }

    // 获得功率挡位
    cap->cap_msg_g.power_limit = power_limit - 30 + 30 * (cap->cap_msg_s.CapVot - 17.0f) / 6.0f;
}
