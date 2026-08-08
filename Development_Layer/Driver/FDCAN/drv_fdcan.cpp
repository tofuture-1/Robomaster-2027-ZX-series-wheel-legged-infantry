#include "drv_fdcan.h"
#include "stm32h7xx_hal_fdcan.h"
#include "string.h"

bool init_finished = false;

Struct_FDCAN_Manage_Object FDCAN1_Manage_Object = {0};
Struct_FDCAN_Manage_Object FDCAN2_Manage_Object = {0};
Struct_FDCAN_Manage_Object FDCAN3_Manage_Object = {0};

// 超级电容发送缓存
uint8_t CAN_Supercap_Tx_Data[8];

/*-----------------私有变量定义-----------------*/
// FDCAN1 发送缓存
uint8_t FDCAN1_0x1fe_Tx_Data[8] = {0};
uint8_t FDCAN1_0x1ff_Tx_Data[8] = {0};
uint8_t FDCAN1_0x200_Tx_Data[8] = {0};
uint8_t FDCAN1_0x2fe_Tx_Data[8] = {0};
uint8_t FDCAN1_0x2ff_Tx_Data[8] = {0};
uint8_t FDCAN1_0x3fe_Tx_Data[8] = {0};
uint8_t FDCAN1_0x4fe_Tx_Data[8] = {0};

// FDCAN2 发送缓存
uint8_t FDCAN2_0x1fe_Tx_Data[8] = {0};
uint8_t FDCAN2_0x1ff_Tx_Data[8] = {0};
uint8_t FDCAN2_0x200_Tx_Data[8] = {0};
uint8_t FDCAN2_0x2fe_Tx_Data[8] = {0};
uint8_t FDCAN2_0x2ff_Tx_Data[8] = {0};
uint8_t FDCAN2_0x3fe_Tx_Data[8] = {0};
uint8_t FDCAN2_0x4fe_Tx_Data[8] = {0};

// FDCAN3 发送缓存
uint8_t FDCAN3_0x1fe_Tx_Data[8] = {0};
uint8_t FDCAN3_0x1ff_Tx_Data[8] = {0};
uint8_t FDCAN3_0x200_Tx_Data[8] = {0};
uint8_t FDCAN3_0x2fe_Tx_Data[8] = {0};
uint8_t FDCAN3_0x2ff_Tx_Data[8] = {0};
uint8_t FDCAN3_0x3fe_Tx_Data[8] = {0};
uint8_t FDCAN3_0x4fe_Tx_Data[8] = {0};

/**
 * @brief 根据 FDCAN 句柄获取对应的管理对象。
 * @param hfdcan FDCAN 外设句柄指针，用于匹配具体的 FDCAN 实例。
 * @return Struct_FDCAN_Manage_Object* 返回匹配到的管理对象指针，未匹配到时返回空指针。
 */
static Struct_FDCAN_Manage_Object *FDCAN_Get_Manage_Object(
    FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == nullptr) {
    return nullptr;
  }

  if (hfdcan->Instance == FDCAN1) {
    return &FDCAN1_Manage_Object;
  } else if (hfdcan->Instance == FDCAN2) {
    return &FDCAN2_Manage_Object;
  } else if (hfdcan->Instance == FDCAN3) {
    return &FDCAN3_Manage_Object;
  }

  return nullptr;
}

/**
 * @brief 清零指定管理对象中的状态统计信息。
 * @param manage_object FDCAN 管理对象指针，用于指定待清零的状态记录。
 */
static void FDCAN_Reset_Status(Struct_FDCAN_Manage_Object *manage_object) {
  if (manage_object == nullptr) {
    return;
  }

  // 直接清空状态统计结构体，避免残留历史状态。
  memset(&manage_object->Status, 0, sizeof(manage_object->Status));
}

/**
 * @brief 更新发送结果统计信息，并记录本次发送时的 FIFO 空闲深度。
 * @param manage_object FDCAN 管理对象指针，用于保存发送统计结果。
 * @param send_status 本次发送接口返回的 HAL 状态值。
 * @param tx_free_level 发送前读取到的 Tx FIFO 空闲深度。
 */
static void FDCAN_Update_Tx_Status(Struct_FDCAN_Manage_Object *manage_object,
                                   HAL_StatusTypeDef send_status,
                                   uint32_t tx_free_level) {
  if (manage_object == nullptr) {
    return;
  }

  manage_object->Status.Last_Tx_Free_Level = tx_free_level;

  if (send_status == HAL_OK) {
    manage_object->Status.Tx_Success_Count++;
    manage_object->Status.Tx_Consecutive_Fail_Count = 0;
  } else {
    manage_object->Status.Tx_Fail_Count++;
    manage_object->Status.Tx_Consecutive_Fail_Count++;

    if (tx_free_level == 0U) {
      manage_object->Status.Tx_Fifo_Full_Count++;
    }
  }
}

/**
 * @brief 初始化指定 FDCAN 实例的滤波器、通知和管理对象信息。
 * @param hfdcan FDCAN 外设句柄指针，用于指定待初始化的硬件实例。
 * @param Callback_Function 接收回调函数指针，用于报文接收后的上层处理。
 */
void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan,
                FDCAN_Call_Back Callback_Function) {
  // 配置范围滤波器，将指定 ID 段报文接收到 FIFO0。
  FDCAN_FilterTypeDef FDCAN_FilterConfig1;

  FDCAN_FilterConfig1.IdType = FDCAN_STANDARD_ID;
  FDCAN_FilterConfig1.FilterIndex = 0;
  FDCAN_FilterConfig1.FilterType = FDCAN_FILTER_RANGE;
  FDCAN_FilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN_FilterConfig1.FilterID1 = 0x201;
  FDCAN_FilterConfig1.FilterID2 = 0x20B;

  HAL_FDCAN_ConfigFilter(hfdcan, &FDCAN_FilterConfig1);

  // 配置全接收掩码滤波器，将其余标准帧接收到 FIFO1。
  FDCAN_FilterTypeDef FDCAN_FilterConfig2;

  FDCAN_FilterConfig2.IdType = FDCAN_STANDARD_ID;
  FDCAN_FilterConfig2.FilterIndex = 1;
  FDCAN_FilterConfig2.FilterType = FDCAN_FILTER_MASK;
  FDCAN_FilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  FDCAN_FilterConfig2.FilterID1 = 0x000;
  FDCAN_FilterConfig2.FilterID2 = 0x000;

  HAL_FDCAN_ConfigFilter(hfdcan, &FDCAN_FilterConfig2);

  Struct_FDCAN_Manage_Object *manage_object = FDCAN_Get_Manage_Object(hfdcan);
  if (manage_object != nullptr) {
    manage_object->FDCAN_Handler = hfdcan;
    manage_object->Callback_Function = Callback_Function;
    FDCAN_Reset_Status(manage_object);
  }

  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT,
                               FDCAN_REJECT);


  HAL_FDCAN_ActivateNotification(hfdcan,
                                 FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                     FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                                     FDCAN_IT_ERROR_WARNING |
                                     FDCAN_IT_ERROR_PASSIVE |
                                     FDCAN_IT_BUS_OFF,
                                 0);

  HAL_FDCAN_Start(hfdcan);
  FDCAN_Update_Bus_Status(hfdcan);
}

/**
 * @brief 通过指定 FDCAN 实例发送一帧数据，并更新发送统计状态。
 * @param hfdcan FDCAN 外设句柄指针，用于指定发送使用的硬件实例。
 * @param ID 标准帧标识符。
 * @param Data 待发送数据缓冲区指针。
 * @param Length 报文 DLC 长度配置值。
 * @return HAL_StatusTypeDef 返回 HAL 层发送结果。
 */
HAL_StatusTypeDef FDCAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID,
                                  uint8_t *Data, uint8_t Length) {
  FDCAN_TxHeaderTypeDef TxHeader;

  TxHeader.Identifier = ID;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = Length;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x00;

  Struct_FDCAN_Manage_Object *manage_object = FDCAN_Get_Manage_Object(hfdcan);
  uint32_t tx_free_level = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
  HAL_StatusTypeDef send_status =
      HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, Data);

  FDCAN_Update_Tx_Status(manage_object, send_status, tx_free_level);
  if (send_status != HAL_OK) {
    FDCAN_Update_Bus_Status(hfdcan);
  }

  return send_status;
}

/**
 * @brief 以实时方式发送报文，当发送 FIFO 已满时直接返回失败。
 * @param hfdcan FDCAN 外设句柄指针，用于指定发送使用的硬件实例。
 * @param ID 标准帧标识符。
 * @param Data 待发送数据缓冲区指针。
 * @param Length 报文 DLC 长度配置值。
 * @return HAL_StatusTypeDef 返回实时发送结果。
 */
HAL_StatusTypeDef FDCAN_Send_Data_Realtime(FDCAN_HandleTypeDef *hfdcan,
                                           uint16_t ID, uint8_t *Data,
                                           uint8_t Length) {
  uint32_t tx_free_level = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
  if (tx_free_level == 0U) {
    FDCAN_Update_Tx_Status(FDCAN_Get_Manage_Object(hfdcan), HAL_ERROR,
                           tx_free_level);
    FDCAN_Update_Bus_Status(hfdcan);
    return HAL_ERROR;
  }

  return FDCAN_Send_Data(hfdcan, ID, Data, Length);
}

/**
 * @brief 刷新指定 FDCAN 实例的协议状态、错误状态和 FIFO 空闲深度。
 * @param hfdcan FDCAN 外设句柄指针，用于指定待更新状态的硬件实例。
 */
void FDCAN_Update_Bus_Status(FDCAN_HandleTypeDef *hfdcan) {
  Struct_FDCAN_Manage_Object *manage_object = FDCAN_Get_Manage_Object(hfdcan);
  if (manage_object == nullptr) {
    return;
  }

  FDCAN_ProtocolStatusTypeDef protocol_status = {0};
  if (HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status) == HAL_OK) {
    manage_object->Status.Last_Protocol_Status = 0U;
    manage_object->Status.Last_Protocol_Status |=
        ((uint32_t)protocol_status.LastErrorCode & 0x7U);
    manage_object->Status.Last_Protocol_Status |=
        (((uint32_t)protocol_status.DataLastErrorCode & 0x7U) << 3);
    manage_object->Status.Last_Protocol_Status |=
        (((uint32_t)protocol_status.Activity & 0x3U) << 6);
    manage_object->Status.Last_Protocol_Status |=
        (((uint32_t)protocol_status.ErrorPassive & 0x1U) << 8);
    manage_object->Status.Last_Protocol_Status |=
        (((uint32_t)protocol_status.Warning & 0x1U) << 9);
    manage_object->Status.Last_Protocol_Status |=
        (((uint32_t)protocol_status.BusOff & 0x1U) << 10);
  }

  manage_object->Status.Last_Error_Status = HAL_FDCAN_GetError(hfdcan);
  manage_object->Status.Last_Tx_Free_Level =
      HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
}

/**
 * @brief 获取指定 FDCAN 实例的状态统计结构体指针。
 * @param hfdcan FDCAN 外设句柄指针，用于匹配对应的管理对象。
 * @return Struct_FDCAN_Manage_Object::Struct_FDCAN_Status* 返回状态结构体指针，未匹配到时返回空指针。
 */
Struct_FDCAN_Manage_Object::Struct_FDCAN_Status *
FDCAN_Get_Status(FDCAN_HandleTypeDef *hfdcan) {
  Struct_FDCAN_Manage_Object *manage_object = FDCAN_Get_Manage_Object(hfdcan);
  if (manage_object == nullptr) {
    return nullptr;
  }

  return &manage_object->Status;
}

/**
 * @brief FDCAN 电机控制 1ms 周期任务回调，用于下发周期性控制报文。
 */
void RTOS_1ms_FDCAN_Motor_Callback() {
  // DJI 电机控制报文

  // 底盘电机控制报文，ID 对应 0x201~0x204。
  // FDCAN_Send_Data_Realtime(&hfdcan1, 0x200, FDCAN1_0x200_Tx_Data,
  //                          FDCAN_DLC_BYTES_8);
  FDCAN_Send_Data_Realtime(&hfdcan2, 0x200, FDCAN2_0x200_Tx_Data,
                           FDCAN_DLC_BYTES_8);
  // FDCAN_Send_Data_Realtime(&hfdcan2, 0x201, FDCAN1_0x200_Tx_Data,
  //                          FDCAN_DLC_BYTES_8);

  // 云台或扩展电机控制报文，ID 对应 0x205~0x20B。
  //FDCAN_Send_Data_Realtime(&hfdcan1, 0x1ff, FDCAN1_0x1ff_Tx_Data,
  //                           FDCAN_DLC_BYTES_8);
}

/**
 * @brief FDCAN FIFO0 接收中断回调，用于读取 FIFO0 中的新报文并触发上层回调。
 * @param hfdcan FDCAN 外设句柄指针，用于标识触发中断的硬件实例。
 * @param RxFifo0ITs FIFO0 中断状态标志。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET)
    return;

  Struct_FDCAN_Manage_Object *manager_obj;

  if(hfdcan->Instance == FDCAN1)
   manager_obj = &FDCAN1_Manage_Object;
  else if(hfdcan->Instance == FDCAN2)
   manager_obj = &FDCAN2_Manage_Object;
  else if(hfdcan->Instance == FDCAN3)
   manager_obj = &FDCAN3_Manage_Object;
  
  HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &manager_obj->Rx_Buffer.Header, manager_obj->Rx_Buffer.Data);

  if (!init_finished)
    return;
  //上层回调函数
  if(manager_obj->Callback_Function != nullptr)
  {
    manager_obj->Callback_Function(&manager_obj->Rx_Buffer);
  }
}

/**
 * @brief FDCAN FIFO1 接收中断回调，用于读取 FIFO1 中的新报文并触发上层回调。
 * @param hfdcan FDCAN 外设句柄指针，用于标识触发中断的硬件实例。
 * @param RxFifo1ITs FIFO1 中断状态标志。
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs) {

  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == RESET)
    return;

  Struct_FDCAN_Manage_Object* manager_obj;

  if(hfdcan->Instance == FDCAN1)
   manager_obj = &FDCAN1_Manage_Object;
  else if(hfdcan->Instance == FDCAN2)
   manager_obj = &FDCAN2_Manage_Object;
  else if(hfdcan->Instance == FDCAN3)
   manager_obj = &FDCAN3_Manage_Object;

  HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO1, &manager_obj->Rx_Buffer.Header, manager_obj->Rx_Buffer.Data);
  if (!init_finished)
    return;

  //上层回调函数
  if(manager_obj->Callback_Function != nullptr)
  {
    manager_obj->Callback_Function(&manager_obj->Rx_Buffer);
  }
}


/**
 * @brief FDCAN 错误状态中断回调，用于记录 Warning、Error Passive 与 Bus-Off 事件并刷新总线状态。
 * @param hfdcan FDCAN 外设句柄指针，用于标识触发错误状态中断的硬件实例。
 * @param ErrorStatusITs 错误状态中断标志位，用于区分 Warning、Error Passive 与 Bus-Off 事件。
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs) {
  Struct_FDCAN_Manage_Object *manage_object = FDCAN_Get_Manage_Object(hfdcan);
  if (manage_object != nullptr) {
    manage_object->Status.Last_Error_Status = ErrorStatusITs;
  }

  // Warning 与 Error Passive 只做状态采样，不在该阶段直接重启总线。
  if ((ErrorStatusITs & (FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE)) !=
      0U) {
    FDCAN_Update_Bus_Status(hfdcan);
  }

  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U) {
    if (manage_object != nullptr) {
      manage_object->Status.Bus_Off_Count++;
    }

    HAL_FDCAN_Stop(hfdcan);

    if (HAL_FDCAN_Start(hfdcan) == HAL_OK) {

      HAL_FDCAN_ActivateNotification(hfdcan,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                         FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                                         FDCAN_IT_ERROR_WARNING |
                                         FDCAN_IT_ERROR_PASSIVE |
                                         FDCAN_IT_BUS_OFF,
                                     0);
    }
  }

  FDCAN_Update_Bus_Status(hfdcan);
}
