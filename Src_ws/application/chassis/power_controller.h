#pragma once

#include "super_cap.h"

/**
 * @brief cap control task, run in chassis task
 *
 * @param cap
 * @param power_buffer
 * @param power_limit
 * @param switch_from_user
 */
void CapController(SuperCapInstance *cap, uint16_t power_buffer, uint16_t power_limit, uint8_t switch_from_user);