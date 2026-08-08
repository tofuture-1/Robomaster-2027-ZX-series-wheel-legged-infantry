// Copyright (C) 2022 ChenJun 
// Copyright (C) 2024 Zheng Yu 
// Licensed under the Apache-2.0 License. 

#ifndef PC_PACKET_H
#define PC_PACKET_H

#include <stdint.h>
#include "drv_usb.h"

// 机器人发送的数据包（下位机发送，上位机接收）
struct PC_SendPacket
{
    uint8_t head[2];           // 0x5A ֡
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
}__attribute__((packed));

// 机器人接收的数据包（下位机接收，上位机发送）- 使用 control_byte
struct PC_ReceivePacket{
    uint8_t header[2];         // 'S' + 'P' 
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
}__attribute__((packed));


#endif // PC_PACKET_H
