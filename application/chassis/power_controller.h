#pragma once

#include "super_cap.h"

/**
 * @brief power control task, run in chassis task
 * 
 * @param cap 
 * @param power_buffer 
 * @param power_limit 
 * @param switch_from_user 
 */
void PowerController(SuperCapInstance *cap, uint16_t power_buffer, uint16_t power_limit, uint8_t switch_from_user);