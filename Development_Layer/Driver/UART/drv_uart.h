//
// Created by 谭恩泽 on 2025/10/24.
//

#ifndef C_BOARD_DRV_UART_H
#define C_BOARD_DRV_UART_H


#include "usart.h"
#include "stm32h7xx_hal.h"

#define UART_DEVICE_MAX_NUM 10       // 最大串口数量
#define UART_BUFFER_SIZE 300        // 缓冲区深度
#define UART_SLOT_COUNT 2           // 双缓冲

typedef void (*UART_Call_Back)(uint8_t *Buffer, uint16_t Length);

struct Struct_UART_Manage_Object {
    UART_HandleTypeDef *huart;
    uint8_t Rx_Buffer[UART_SLOT_COUNT][UART_BUFFER_SIZE];
    uint8_t Rx_Slot_Index;
    uint16_t Max_Rx_Length;
    UART_Call_Back Callback_Function;
};

// 核心接口
Struct_UART_Manage_Object* UART_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function, uint16_t Max_Rx_Length);
uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length);
void UART_Reinit(UART_HandleTypeDef *huart);


#endif // C_BOARD_DRV_UART_H
