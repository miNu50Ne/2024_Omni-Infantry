/**
 * @file master_process.c
 * @author smoaflie
 * @brief
 * @version 0.1
 * @date 2024-3-26
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "master_process.h"
#include "daemon.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "bsp_usb.h"
#include <stdint.h>

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 */
static void HostOfflineCallback(void *instance)
{
    HostInstance *_instance = (HostInstance *)instance;
    switch (_instance->comm_mode) {
        case HOST_USART:
            USARTServiceInit(_instance->comm_instance);
            break;
        case HOST_VCP:
            LOGWARNING("[vision] vision offline, restart communication.");
            USBRefresh();
            break;
    }
}

HostInstance *HostInit(HostInstanceConf *host_conf)
{
    HostInstance *hostinstance_init = (HostInstance *)malloc(sizeof(HostInstance));
    memset(hostinstance_init, 0, sizeof(HostInstance));
    hostinstance_init->comm_mode = host_conf->comm_mode;
    hostinstance_init->RECV_SIZE = host_conf->RECV_SIZE;

    switch (host_conf->comm_mode) {
        case HOST_USART:
            USART_Init_Config_s usart_conf;
            usart_conf.module_callback       = host_conf->callback;
            usart_conf.recv_buff_size        = host_conf->RECV_SIZE;
            usart_conf.usart_handle          = host_conf->usart_handle;
            hostinstance_init->comm_instance = USARTRegister(&usart_conf);
            break;
        case HOST_VCP:
            USB_Init_Config_s usb_conf       = {.rx_cbk = host_conf->callback};
            hostinstance_init->comm_instance = USBInit(usb_conf);
            break;
        default:
            while (1)
                LOGERROR("[master_process]You must select correct mode for HOST.");
    }

    // 为上位机实例注册守护进程
    Daemon_Init_Config_s daemon_conf = {
        .callback     = HostOfflineCallback, // 离线时调用的回调函数,会重启实例
        .owner_id     = hostinstance_init,
        .reload_count = 10,
    };
    hostinstance_init->daemon = DaemonRegister(&daemon_conf);

    return hostinstance_init;
}

/**
 * @brief 发送函数
 *
 * @param instance 上位机实例
 * @param sendbuf  发送内容
 * @param tx_len 发送长度
 *
 */
void HostSend(HostInstance *instance, uint8_t *send_buf, uint16_t tx_len)
{
    switch (instance->comm_mode) {
        case HOST_USART:
            // todo：是否需要对发送方式进行定制（即轮询/中断/DMA三种模式）
            USARTSend((USARTInstance *)instance->comm_instance, send_buf, tx_len, USART_TRANSFER_DMA);
            break;
        case HOST_VCP:
            USBTransmit(send_buf, tx_len);
            break;
    }
}
