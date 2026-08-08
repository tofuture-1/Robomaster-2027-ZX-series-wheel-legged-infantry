#ifndef DRV_FDCAN_H
#define DRV_FDCAN_H

#include "stm32h7xx_hal.h"//hzy-显式声明增加可读性及稳定性
#include "fdcan.h"
#include "cstring"

extern bool init_finished;

/*-----------------私有变量声明-----------------*/

// FDCAN1 发送缓存
extern uint8_t FDCAN1_0x1fe_Tx_Data[8];
extern uint8_t FDCAN1_0x1ff_Tx_Data[8];
extern uint8_t FDCAN1_0x200_Tx_Data[8];
extern uint8_t FDCAN1_0x2fe_Tx_Data[8];
extern uint8_t FDCAN1_0x2ff_Tx_Data[8];
extern uint8_t FDCAN1_0x3fe_Tx_Data[8];
extern uint8_t FDCAN1_0x4fe_Tx_Data[8];

// FDCAN2 发送缓存
extern uint8_t FDCAN2_0x1fe_Tx_Data[8];
extern uint8_t FDCAN2_0x1ff_Tx_Data[8];
extern uint8_t FDCAN2_0x200_Tx_Data[8];
extern uint8_t FDCAN2_0x2fe_Tx_Data[8];
extern uint8_t FDCAN2_0x2ff_Tx_Data[8];
extern uint8_t FDCAN2_0x3fe_Tx_Data[8];
extern uint8_t FDCAN2_0x4fe_Tx_Data[8];

// FDCAN3 发送缓存
extern uint8_t FDCAN3_0x1fe_Tx_Data[8];
extern uint8_t FDCAN3_0x1ff_Tx_Data[8];
extern uint8_t FDCAN3_0x200_Tx_Data[8];
extern uint8_t FDCAN3_0x2fe_Tx_Data[8];
extern uint8_t FDCAN3_0x2ff_Tx_Data[8];
extern uint8_t FDCAN3_0x3fe_Tx_Data[8];
extern uint8_t FDCAN3_0x4fe_Tx_Data[8];

/* function prototypes -------------------------------------------------------*/

/**
 * @brief FDCAN 接收缓存结构体，用于保存最近一次接收到的报文头与数据。
 */
struct Struct_FDCAN_Rx_Buffer
{
    FDCAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
};

/**
 * @brief FDCAN 接收回调函数类型定义，用于在接收到报文后向上层通知数据。
 * @param rx_buffer 指向接收缓存结构体的指针，包含报文头和 8 字节数据区。
 */
typedef void (*FDCAN_Call_Back)(Struct_FDCAN_Rx_Buffer *rx_buffer);

/**
 * @brief FDCAN 管理对象，用于统一维护句柄、接收缓存、回调函数及运行状态。
 */
struct Struct_FDCAN_Manage_Object
{
    FDCAN_HandleTypeDef *FDCAN_Handler;
    Struct_FDCAN_Rx_Buffer Rx_Buffer;
    FDCAN_Call_Back Callback_Function;

    /**
     * @brief FDCAN 运行状态统计信息，用于记录发送结果、错误状态及总线异常信息。
     */
    struct Struct_FDCAN_Status
    {
        uint32_t Tx_Success_Count;
        uint32_t Tx_Fail_Count;
        uint32_t Tx_Fifo_Full_Count;
        uint32_t Tx_Consecutive_Fail_Count;
        uint32_t Bus_Off_Count;
        uint32_t Last_Tx_Free_Level;
        uint32_t Last_Error_Status;
        uint32_t Last_Protocol_Status;
    } Status;
};

extern Struct_FDCAN_Manage_Object FDCAN1_Manage_Object;
extern Struct_FDCAN_Manage_Object FDCAN2_Manage_Object;
extern Struct_FDCAN_Manage_Object FDCAN3_Manage_Object;

/**
 * @brief 初始化指定的 FDCAN 外设，并注册对应的接收回调函数。
 * @param hfdcan FDCAN 外设句柄指针，用于指定待初始化的硬件实例。
 * @param Callback_Function 接收完成后的回调函数指针，可为上层模块提供报文处理入口。
 */
void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan, FDCAN_Call_Back Callback_Function);

/**
 * @brief 通过指定的 FDCAN 外设发送一帧标准数据报文。
 * @param hfdcan FDCAN 外设句柄指针，用于指定发送使用的硬件实例。
 * @param ID 标准帧标识符。
 * @param Data 待发送的数据缓冲区指针。
 * @param Length 待发送的数据长度，需与 HAL 定义的 DLC 长度配置保持一致。
 * @return HAL_StatusTypeDef 返回 HAL 层发送结果。
 */
HAL_StatusTypeDef FDCAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Data, uint8_t Length);

/**
 * @brief 立即发送一帧 FDCAN 数据报文，并在发送失败时返回明确的状态结果。
 * @param hfdcan FDCAN 外设句柄指针，用于指定发送使用的硬件实例。
 * @param ID 标准帧标识符。
 * @param Data 待发送的数据缓冲区指针。
 * @param Length 待发送的数据长度，需与 HAL 定义的 DLC 长度配置保持一致。
 * @return HAL_StatusTypeDef 返回实时发送结果，发送异常时通常返回 HAL_ERROR。
 */
HAL_StatusTypeDef FDCAN_Send_Data_Realtime(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Data, uint8_t Length);

/**
 * @brief 更新指定 FDCAN 外设的总线运行状态，并刷新错误统计信息。
 * @param hfdcan FDCAN 外设句柄指针，用于指定待更新状态的硬件实例。
 */
void FDCAN_Update_Bus_Status(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief 获取指定 FDCAN 外设对应的状态统计结构体指针。
 * @param hfdcan FDCAN 外设句柄指针，用于匹配对应的管理对象。
 * @return Struct_FDCAN_Manage_Object::Struct_FDCAN_Status* 返回状态结构体指针，未匹配到时返回空指针。
 */
Struct_FDCAN_Manage_Object::Struct_FDCAN_Status *FDCAN_Get_Status(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief FDCAN 电机控制 1ms 周期任务回调函数，用于执行周期性报文处理。
 */
void RTOS_1ms_FDCAN_Motor_Callback();

#endif // DRV_FDCAN_H
