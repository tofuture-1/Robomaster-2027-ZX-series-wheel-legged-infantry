//
// Created by 谭恩泽 on 2026/4/7.
//

#include "g_c_communicate.h"
#include <string.h>


void G_C_Communicate::Init(FDCAN_HandleTypeDef *hfdcan)
{
    this->hfdcan_ptr = hfdcan;
}


void G_C_Communicate::CAN_RxCpltCallback(Struct_FDCAN_Rx_Buffer *Rx_Buffer)
{
    Flag++;
    Data_Process(Rx_Buffer);
}


void G_C_Communicate::Data_Process(Struct_FDCAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.Identifier & 0x7FF)
    {
    case 0x100:
    {
        this->ReceivePacket.Enable = Rx_Buffer->Data[0] & 0x01;
        this->ReceivePacket.Robot_Level = Rx_Buffer->Data[1] & 0x07;
        this->ReceivePacket.Self_Booster_Heat_Max = ((uint16_t)Rx_Buffer->Data[2] << 8) | Rx_Buffer->Data[3];
        this->ReceivePacket.Self_Booster_Heat_CD = ((uint16_t)Rx_Buffer->Data[4] << 8) | Rx_Buffer->Data[5];
        this->ReceivePacket.Booster_17mm_1_Heat = ((uint16_t)Rx_Buffer->Data[6] << 8) | Rx_Buffer->Data[7];

        this->Data.Self_Booster_Heat_Max = (float)this->ReceivePacket.Self_Booster_Heat_Max;
        this->Data.Self_Booster_Heat_CD = (float)this->ReceivePacket.Self_Booster_Heat_CD;
        this->Data.Booster_17mm_1_Heat = (float)this->ReceivePacket.Booster_17mm_1_Heat;
        break;
    }
    case 0x101:
    {
        this->ReceivePacket.Booster_17mm_Speed = ((uint16_t)Rx_Buffer->Data[0] << 8) | Rx_Buffer->Data[1];
        {
            uint32_t raw = ((uint32_t)Rx_Buffer->Data[2] << 24) |
                           ((uint32_t)Rx_Buffer->Data[3] << 16) |
                           ((uint32_t)Rx_Buffer->Data[4] << 8)  |
                           Rx_Buffer->Data[5];
            memcpy((void *)&this->ReceivePacket.Chassis_Omega, &raw, 4);
        }
				
				//活动数据lkadjflkad
				this->Data.Booster_17mm_Speed = (float)this->ReceivePacket.Booster_17mm_Speed;
        this->Data.Chassis_Omega = this->ReceivePacket.Chassis_Omega;
        break;
    }
    default:
        break;
    }
}


/**
 * @brief 100ms 定时器回调，用于在线检测
 * 
 */
void G_C_Communicate::RTOS_100ms_Alive_Callback()
{
    if (Flag == Pre_Flag)
    {
        G_C_Status = G_C_Status_DISABLE;
    }
    else
    {
        G_C_Status = G_C_Status_ENABLE;
        Pre_Flag = Flag;
    }
}


/**
 * @brief 用于整理好通信数据并发送的函数
 * 
 */
 void G_C_Communicate::RTOS_1ms_Calculate_Callback()
 {
        uint8_t data_1[8];
        uint8_t data_2[8];
        memcpy(data_1, &this->SendPacket, 8);
        memcpy(data_2, (uint8_t *)&this->SendPacket + 8, 4);
        data_2[4] = this->SendPacket.is_gyro;
        data_2[5] = 0;
        data_2[6] = 0;
        data_2[7] = 0;

        FDCAN_Send_Data(this->hfdcan_ptr, 0x110, data_1, 8);
        FDCAN_Send_Data(this->hfdcan_ptr, 0x111, data_2, 8);
 }
