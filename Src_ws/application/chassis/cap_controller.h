#pragma once

#include "super_cap.h"
#include "ramp.h"
/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 * @param
 * @param
 *
 */
float LimitChassisOutput(uint16_t power_buffer, uint16_t power_limit, ramp_t *s_ramp_);

/**
 * @brief 开启超电后提高功率上限
 *
 */
float SuperLimitOutput(float cap_voltage, ramp_t *s_ramp_);

/**
 * @brief cap control task, run in chassis task
 *
 * @param cap
 * @param power_buffer
 * @param power_limit
 * @param switch_from_user
 */
void cap_controller(SuperCapInstance *cap, uint16_t power_buffer, uint16_t power_limit, uint8_t switch_from_user);