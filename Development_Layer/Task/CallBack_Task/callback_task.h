//
// Created by 谭恩泽 on 2025/10/30.
//

#ifndef C_BOARD_CALLBACK_TASK_H
#define C_BOARD_CALLBACK_TASK_H


#include "rbt_class.h"
#include "drv_fdcan.h"

extern CLASS_ROBOT robot;

void Device_FDCAN1_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage);

void Device_FDCAN2_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage);

void Device_FDCAN3_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage);

void USART1_RX_Callback(uint8_t *Buffer, uint16_t Length);

void DR16_UART5_Callback(uint8_t *Buffer, uint16_t Length);

void VOFA_UART6_Callback(uint8_t *Buffer, uint16_t Length);

void BMI088_SPI2_Callback(uint16_t Length);

void TIM6_Robot_1ms_Callback();
    
void USART10_RX_Callback(uint8_t *Buffer, uint16_t Length);


#endif //C_BOARD_CALLBACK_TASK_H
