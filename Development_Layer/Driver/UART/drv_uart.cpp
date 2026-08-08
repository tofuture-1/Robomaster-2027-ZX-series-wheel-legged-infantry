//
// Created by 谭恩泽 on 2025/10/24.
//

#include "drv_uart.h"
#include "string.h"


// 静态内存池，避免动态分配产生碎片
static Struct_UART_Manage_Object UART_Pool[UART_DEVICE_MAX_NUM];

static Struct_UART_Manage_Object* UART1_Object = nullptr;
static Struct_UART_Manage_Object* UART2_Object = nullptr;
static Struct_UART_Manage_Object* UART3_Object = nullptr;
static Struct_UART_Manage_Object* UART4_Object = nullptr;
static Struct_UART_Manage_Object* UART5_Object = nullptr;
static Struct_UART_Manage_Object* UART6_Object = nullptr;
static Struct_UART_Manage_Object* UART7_Object = nullptr;
static Struct_UART_Manage_Object* UART8_Object = nullptr;
static Struct_UART_Manage_Object* UART9_Object = nullptr;
static Struct_UART_Manage_Object* UART10_Object = nullptr;
static uint8_t Registered_Count = 0;

extern bool init_finished;

Struct_UART_Manage_Object* UART_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function, uint16_t Max_Rx_Length) {
    if (huart == nullptr || Registered_Count >= UART_DEVICE_MAX_NUM) return nullptr;

    Struct_UART_Manage_Object* obj = &UART_Pool[Registered_Count];
    obj->huart = huart;
    obj->Callback_Function = Callback_Function;
    obj->Max_Rx_Length = (Max_Rx_Length > UART_BUFFER_SIZE) ? UART_BUFFER_SIZE : Max_Rx_Length;
    obj->Rx_Slot_Index = 0;

    switch ((uint32_t)huart->Instance) {
        case USART1_BASE: UART1_Object = obj; break;
        case USART2_BASE: UART2_Object = obj; break;
        case USART3_BASE: UART3_Object = obj; break;
        case UART4_BASE:  UART4_Object = obj; break;
        case UART5_BASE:  UART5_Object = obj; break;
        case USART6_BASE: UART6_Object = obj; break;
        case UART7_BASE:  UART7_Object = obj; break;
        case UART8_BASE:  UART8_Object = obj; break;
        case UART9_BASE:  UART9_Object = obj; break;
        case USART10_BASE: UART10_Object = obj; break;
    }
    Registered_Count++;

    HAL_UARTEx_ReceiveToIdle_DMA(huart, obj->Rx_Buffer[obj->Rx_Slot_Index], obj->Max_Rx_Length);
    return obj;
}

/**
 * @brief UART数据分发服务
 * 
 * @param obj 
 * @param Size 
 */
static void UART_Dispatch_Service(Struct_UART_Manage_Object* obj, uint16_t Size) {
    uint8_t* last_rx_data_ptr = obj->Rx_Buffer[obj->Rx_Slot_Index];
    obj->Rx_Slot_Index = (obj->Rx_Slot_Index + 1) % UART_SLOT_COUNT;

    // 立即重启DMA，指向新槽位
    HAL_UARTEx_ReceiveToIdle_DMA(obj->huart, obj->Rx_Buffer[obj->Rx_Slot_Index], obj->Max_Rx_Length);


    if (obj->Callback_Function != nullptr) {
        obj->Callback_Function(last_rx_data_ptr, Size);
    }
}

/**
 * @brief UART接收完成回调函数
 * 
 * @param huart 
 * @param Size 
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (!init_finished) return;
    
    // 半传输事件不触发缓冲切换：DMA未完成，切换到另一缓冲区会导致TC时取错buffer
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
        return;
    }
    
    Struct_UART_Manage_Object* obj = nullptr;
    switch ((uint32_t)huart->Instance) {
        case USART1_BASE: obj = UART1_Object; break;
        case USART2_BASE: obj = UART2_Object; break;
        case USART3_BASE: obj = UART3_Object; break;
        case UART4_BASE:  obj = UART4_Object; break;
        case UART5_BASE:  obj = UART5_Object; break;
        case USART6_BASE: obj = UART6_Object; break;
        case UART7_BASE:  obj = UART7_Object; break;
        case UART8_BASE:  obj = UART8_Object; break;
        case UART9_BASE:  obj = UART9_Object; break;
        case USART10_BASE: obj = UART10_Object; break;
    }
    if (obj) UART_Dispatch_Service(obj, Size);
}

/**
 * @brief UART错误回调函数
 * 
 * @param huart 
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    Struct_UART_Manage_Object* obj = nullptr;
    switch ((uint32_t)huart->Instance) {
        case USART1_BASE: obj = UART1_Object; break;
        case USART2_BASE: obj = UART2_Object; break;
        case USART3_BASE: obj = UART3_Object; break;
        case UART4_BASE:  obj = UART4_Object; break;
        case UART5_BASE:  obj = UART5_Object; break;
        case USART6_BASE: obj = UART6_Object; break;
        case UART7_BASE:  obj = UART7_Object; break;
        case UART8_BASE:  obj = UART8_Object; break;
        case UART9_BASE:  obj = UART9_Object; break;
        case USART10_BASE: obj = UART10_Object; break;
    }
    if (obj) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, obj->Rx_Buffer[obj->Rx_Slot_Index], obj->Max_Rx_Length);
    }
}

uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length) {
    return HAL_UART_Transmit_DMA(huart, Data, Length);
}

/**
 * @brief 重新初始化UART
 * 
 */
void UART_Reinit(UART_HandleTypeDef *huart) {
   Struct_UART_Manage_Object* obj = nullptr;
   switch ((uint32_t)huart->Instance) {
       case USART1_BASE: obj = UART1_Object; break;
       case USART2_BASE: obj = UART2_Object; break;
       case USART3_BASE: obj = UART3_Object; break;
       case UART4_BASE:  obj = UART4_Object; break;
       case UART5_BASE:  obj = UART5_Object; break;
       case USART6_BASE: obj = UART6_Object; break;
       case UART7_BASE:  obj = UART7_Object; break;
       case UART8_BASE:  obj = UART8_Object; break;
       case UART9_BASE:  obj = UART9_Object; break;
       case USART10_BASE: obj = UART10_Object; break;
   }
   
   if (obj == nullptr) return;

   __disable_irq();

   HAL_UART_DMAStop(huart);

   __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF | UART_CLEAR_IDLEF);
   __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_RTOF | UART_CLEAR_CTSF | UART_CLEAR_LBDF);

   huart->RxState = HAL_UART_STATE_READY;
   huart->ErrorCode = HAL_UART_ERROR_NONE;

   if (huart->hdmarx != NULL) {
       HAL_DMA_Abort(huart->hdmarx);
       huart->hdmarx->State = HAL_DMA_STATE_READY;
       huart->hdmarx->ErrorCode = HAL_DMA_ERROR_NONE;
   }

   obj->Rx_Slot_Index = 0;

   memset(obj->Rx_Buffer, 0, sizeof(obj->Rx_Buffer));

   __enable_irq();

   HAL_UARTEx_ReceiveToIdle_DMA(huart, obj->Rx_Buffer[obj->Rx_Slot_Index], obj->Max_Rx_Length);
}
