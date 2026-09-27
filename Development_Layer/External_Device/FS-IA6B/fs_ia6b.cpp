//
// Created by 24158 on 2026/6/17.
//

#include "fs_ia6b.h"
#include <stdio.h>

// [CN] 静态指针，用于回调函数中访问具体的类实例
static Class_FS_IA6B* Global_FS_IA6B_Instance = nullptr;

// [CN] 驱动层所需的回调函数桥接
static void FS_IA6B_Bridge_Callback(uint8_t* buf, uint16_t len) {
    if (Global_FS_IA6B_Instance) Global_FS_IA6B_Instance->UART_RxCpltCallback(buf,len);
}
//TODO：这部分内容还需要研究，貌似有：只支持一个实例、生命周期、Data_Process中就存在的竞态等问题
/**
 * @brief 遥控器FS_I6X/对应接收机初始化
 *
 * @param huart 指定的UART或者usart
 */
void Class_FS_IA6B::Init(UART_HandleTypeDef *huart)
{
    Global_FS_IA6B_Instance = this;
    UART_Manage_Object = UART_Init(huart, FS_IA6B_Bridge_Callback, 32);//Ibus一帧定长貌似为32

}

void Class_FS_IA6B::Data_Process(uint8_t *Rx_Frame ){

    uint16_t	calc_sum = 0;
    uint16_t 	ori_sum;
    uint8_t  	i;

    if( Rx_Frame[0] != 0x20 )
        return;
    if( Rx_Frame[1] != 0x40 )
        return;

    for( i = 0; i < 30; i++ )
        calc_sum += Rx_Frame[i];
    calc_sum ^= 0xFFFF;
    ori_sum = Rx_Frame[30] + (Rx_Frame[31]<<8);

    // printf( "calc_sum: %04x %04x\n", (unsigned)calc_sum, (unsigned)ori_sum );
    if( calc_sum != ori_sum )
        return;

    // IBUS 协议：通道值 12 位 (0~4095)
    for (i = 0; i < 14; i++)
    {
        FS_IA6B_ibus_msg.ch[i] = Rx_Frame[i * 2 + 2] + ((Rx_Frame[i * 2 + 3] & 0x0F) << 8);
    }

    // 通道 15~18（FS-IA6B 无输出）
    for (i = 14; i < 18; i++)
    {
        FS_IA6B_ibus_msg.ch[i] = 0;
    }

}

void Class_FS_IA6B::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    Flag += 1;// TODO:需要实现相关的前置判定，才能实现跳过该步骤以达成掉线保护，具体参考Class_VTM_RX中的实现？但貌似dr16中的未使用类似”接收不全“的判定？

    Data_Process(Rx_Data);

}

void Class_FS_IA6B::RTOS_100ms_Alive_Callback()
{
    if (Flag == Pre_Flag)
    {
        FS_IA6B_Status = FS_IA6B_Status_DISABLE;

        static uint8_t cnt = 0;
        cnt++;
        if (cnt > 5)
        {
            cnt = 0;
            UART_Reinit(UART_Manage_Object->huart);
        }
    }
    else
    {
        FS_IA6B_Status = FS_IA6B_Status_ENABLE;
    }

    Pre_Flag = Flag;
}
