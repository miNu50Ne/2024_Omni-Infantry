/**
 * @Author: HDC h2019dc@outlook.com
 * @Date: 2023-09-08 16:47:43
 * @LastEditors: HDC h2019dc@outlook.com
 * @LastEditTime: 2023-10-24 20:01:02
 * @FilePath: \2024_Control_New_Framework_Base-dev-all\modules\super_cap\super_cap.c
 * @Description:
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:29:49
 */
#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"

static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void SuperCapRxCallback(CANInstance *_instance)
{
    struct _packed {
        uint16_t chassis_power;
        uint16_t cap_voltage;
        uint16_t chassis_voltage;
        uint8_t enabled;
        uint8_t unused;
    } *rxbuff;
    SuperCap_Msg_s *Msg;
    rxbuff = _instance->rx_buff;

    Msg                               = &super_cap_instance->cap_msg_s;
    Msg->chassis_power_from_cap       = uint_to_float(rxbuff->chassis_power, 0.0, 500.0, 16);
    Msg->chassis_voltage_from_cap     = uint_to_float(rxbuff->chassis_voltage, 0.0, 50.0, 16);
    Msg->CapVot                       = uint_to_float(rxbuff->cap_voltage, 0.0, 50.0, 16);
    Msg->SuperCap_open_flag_from_real = rxbuff->enabled;
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    super_cap_instance = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance, 0, sizeof(SuperCapInstance));

    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance->can_ins                     = CANRegister(&supercap_config->can_config);
    return super_cap_instance;
}

void SuperCapSend(SuperCapInstance *instance, uint8_t *data)
{
    memcpy(instance->can_ins->tx_buff, data, 8);
    CANTransmit(instance->can_ins, 1);
}

SuperCap_Msg_s SuperCapGet(SuperCapInstance *instance)
{
    return instance->cap_msg_s;
}