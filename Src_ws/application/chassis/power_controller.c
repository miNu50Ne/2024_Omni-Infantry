#include "super_cap.h"
#include "ramp.h"
#include "robot_def.h"
#include "power_calc.h"
#include <stdint.h>

static ramp_t *super_ramp;
static float Power_Output;
/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 * @param
 * @param
 *
 */
static void LimitChassisOutput(uint16_t power_buffer, uint16_t power_limit)
{
    static float Plimit;

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

    Power_Output = power_limit;
    // Power_Output = power_limit - 10 + 20 * Plimit;
    PowerControlupdate(Power_Output, REDUCTION_RATIO_WHEEL);

    ramp_init(super_ramp, 300);
}

/**
 * @brief 开启超电后提高功率上限
 *
 */
static void SuperLimitOutput(float cap_voltage)
{
    static float power_output;
    Power_Output = (power_output + (250 - 20 + 40 * (cap_voltage - 17.0f) / 6.0f - power_output) * ramp_calc(super_ramp));
    PowerControlupdate(Power_Output, REDUCTION_RATIO_WHEEL);
    power_output = Power_Output;
}

/**
 * @brief 超电开关
 *
 *
 */
uint8_t Super_Voltage_Allow_Flag;

void PowerController(SuperCapInstance *cap, uint16_t power_buffer, uint16_t power_limit, uint8_t switch_from_user)
{
    SuperCap_State_e SuperCap_state = SUPER_STATE_LOW;
    // 状态机逻辑,滞回
    switch (SuperCap_state) {
        case SUPER_STATE_LOW:
            if (cap->cap_msg_s.CapVot > SUPER_VOLTAGE_THRESHOLD_HIGH) {
                SuperCap_state = SUPER_STATE_HIGH;
            }
            break;
        case SUPER_STATE_HIGH:
            if (cap->cap_msg_s.CapVot < SUPER_VOLTAGE_THRESHOLD_LOW) {
                SuperCap_state = SUPER_STATE_LOW;
            }
            break;
        default:
            SuperCap_state = SUPER_STATE_LOW;
            break;
    }

    // 小于12V关闭
    if (SuperCap_state == SUPER_STATE_LOW) {
        Super_Voltage_Allow_Flag = SUPER_VOLTAGE_CLOSE;
    } else if (SuperCap_state == SUPER_STATE_HIGH) {
        Super_Voltage_Allow_Flag = SUPER_VOLTAGE_OPEN;
    } else {
        // none
    }

    // User允许开启电容 且 电压充足
    switch (switch_from_user) {
        case SUPER_USER_OPEN:
            cap->cap_msg_g.enabled = SUPER_CMD_OPEN;
            SuperLimitOutput(cap->cap_msg_s.CapVot);
            break;
        case SUPER_USER_CLOSE:
            cap->cap_msg_g.enabled = SUPER_CMD_CLOSE;
            LimitChassisOutput(power_buffer, power_limit);
            break;
    }

    // 获得功率挡位
    cap->cap_msg_g.power_limit = power_limit - 30 + 30 * (cap->cap_msg_s.CapVot - 17.0f) / 6.0f;
}
