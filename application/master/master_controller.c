/**
 * @file master_controller.c
 * @author smoaflie
 * @brief
 * @version 0.1
 * @date 2024-3-26
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "message_center.h"
#include "robot_def.h"
#include "master_process.h"

#include <stdint.h>
#include <string.h>

static HostInstance *host_nuc;

// // 这里的四元数以wxyz的顺序
static uint8_t master_send_data[9];  // 上位机发送的数据-绝对角度，第9个字节作为识别到目标的标志位
static uint8_t master_recv_data[23]; // 上位机接收的数据-帧头，四元数，机器人id，是否打符

static Publisher_t *master_pub;
static Subscriber_t *master_sub;

static Master_Cmd_s master_cmd_recv;
static Master_Upload_Data_s master_feedback_data;
/**
 * @brief usbj接收回调函数
 *
 */
static void HOST_RECV_CALLBACK()
{
    memcpy(master_send_data, host_nuc->comm_instance, host_nuc->RECV_SIZE);
    master_send_data[8] = 1;
}

void MasterDeviceInit()
{
    HostInstanceConf host_conf = {
        .callback  = HOST_RECV_CALLBACK,
        .comm_mode = HOST_VCP,
        .RECV_SIZE = 8,
    };
    host_nuc = HostInit(&host_conf);
}

void MasterMsgInit()
{
    master_pub = PubRegister("master_feed", sizeof(Master_Upload_Data_s));
    master_sub = SubRegister("master_cmd", sizeof(Master_Cmd_s));
}

void MasterMsgProcess()
{
    memcpy(master_recv_data, &master_cmd_recv, sizeof(Master_Cmd_s));
    for (size_t i = 0; i < 22; i++)
        master_recv_data[22] += master_recv_data[i];

    memcpy(&master_feedback_data.rec_yaw, master_send_data, sizeof(float));
    memcpy(&master_feedback_data.rec_pitch, master_send_data + 4, sizeof(float));
}

void MasterMsgComm()
{
    SubGetMessage(master_sub, &master_cmd_recv);

    HostSend(host_nuc, master_recv_data, 23);

    PubPushMessage(master_pub, (void *)&master_feedback_data);
}