//
// Created by 谭恩泽 on 2025/10/31.
//

#include "dr16.h"



// [CN] 静态指针，用于回调函数中访问具体的类实例
static Class_DR16* Global_DR16_Instance = nullptr;

// [CN] 驱动层所需的回调函数桥接
static void DR16_Bridge_Callback(uint8_t* buf, uint16_t len) {
    if (Global_DR16_Instance) Global_DR16_Instance->UART_RxCpltCallback(buf, len);
}

/**
 * @brief 遥控器DR16初始化
 *
 * @param huart 指定的UART或者usart
 */
void Class_DR16::Init(UART_HandleTypeDef *huart)
{
   Global_DR16_Instance = this;
   UART_Manage_Object = UART_Init(huart, DR16_Bridge_Callback, 18);

}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_DR16::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    // 滑动窗口, 判断遥控器DR16是否在线
    Flag += 1;

    Data_Process(Rx_Data, Length);
}

/**
 * @brief TIM定时器中断定期检测遥控器DR16是否存活
 *
 */
void Class_DR16::RTOS_100ms_Alive_Callback()
{
    // 判断该时间段内是否接收过遥控器DR16数据
    if (Flag == Pre_Flag)
    {
        // 遥控器DR16断开连接
        DR16_Status = DR16_Status_DISABLE;
 
			static uint8_t cnt = 0;
			cnt++;
	     if(cnt > 5)
			 {
				 cnt = 0;
        UART_Reinit(UART_Manage_Object->huart);
			 }
    }
    else
    {
        // 遥控器DR16保持连接
        DR16_Status = DR16_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief 定时器计算函数
 *
 */
void Class_DR16::RTOS_1ms_Calculate_Callback()
{
    Struct_DR16_UART_Data *tmp_buffer = (Struct_DR16_UART_Data *) UART_Manage_Object->Rx_Buffer[(UART_Manage_Object->Rx_Slot_Index+1)%2];

    // 判断拨码触发
    _Judge_Switch(&Data.Left_Switch, tmp_buffer->Switch_1, Pre_UART_Rx_Data.Switch_1);
    _Judge_Switch(&Data.Right_Switch, tmp_buffer->Switch_2, Pre_UART_Rx_Data.Switch_2);

    // 判断鼠标触发
    _Judge_Key(&Data.Mouse_Left_Key, tmp_buffer->Mouse_Left_Key, Pre_UART_Rx_Data.Mouse_Left_Key);
    _Judge_Key(&Data.Mouse_Right_Key, tmp_buffer->Mouse_Right_Key, Pre_UART_Rx_Data.Mouse_Right_Key);

    // 判断键盘触发
    for (int i = 0; i < 16; i++)
    {
        _Judge_Key(&Data.Keyboard_Key[i], ((tmp_buffer->Keyboard_Key) >> i) & 0x1, ((Pre_UART_Rx_Data.Keyboard_Key) >> i) & 0x1);
    }

    // 保留数据
    memcpy(&Pre_UART_Rx_Data, tmp_buffer, 18 * sizeof(uint8_t));
}

/**
 * @brief 数据处理过程
 *
 */
void Class_DR16::Data_Process(uint8_t *Rx_Frame, uint16_t Length)
{
    // 数据处理过程
    Struct_DR16_UART_Data *tmp_buffer = (Struct_DR16_UART_Data *) Rx_Frame;

    // 摇杆信息
    Data.Right_X = (tmp_buffer->Channel_0 - Rocker_Offset) / Rocker_Num;
    Data.Right_Y = (tmp_buffer->Channel_1 - Rocker_Offset) / Rocker_Num;
    Data.Left_X = (tmp_buffer->Channel_2 - Rocker_Offset) / Rocker_Num;
    Data.Left_Y = (tmp_buffer->Channel_3 - Rocker_Offset) / Rocker_Num;

    // 鼠标信息
    Data.Mouse_X = tmp_buffer->Mouse_X / 32768.0f;
    Data.Mouse_Y = tmp_buffer->Mouse_Y / 32768.0f;
    Data.Mouse_Z = tmp_buffer->Mouse_Z / 32768.0f;

        // 判断拨码触发
    _Judge_Switch(&Data.Left_Switch, tmp_buffer->Switch_1, Pre_UART_Rx_Data.Switch_1);
    _Judge_Switch(&Data.Right_Switch, tmp_buffer->Switch_2, Pre_UART_Rx_Data.Switch_2);

    // 判断鼠标触发
    _Judge_Key(&Data.Mouse_Left_Key, tmp_buffer->Mouse_Left_Key, Pre_UART_Rx_Data.Mouse_Left_Key);
    _Judge_Key(&Data.Mouse_Right_Key, tmp_buffer->Mouse_Right_Key, Pre_UART_Rx_Data.Mouse_Right_Key);

    // 左前轮信息
    Data.Yaw = (tmp_buffer->Channel_Yaw - Rocker_Offset) / Rocker_Num;
}

/**
 * @brief 判断拨动开关状态
 *
 */
void Class_DR16::_Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status)
{
    // 带触发的判断
    switch (Pre_Status)
    {
    case (DR16_SWITCH_UP):
    {
        switch (Status)
        {
        case (DR16_SWITCH_UP):
        {
            *Switch = DR16_Switch_Status_UP;

            break;
        }
        case (DR16_SWITCH_DOWN):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;

            break;
        }
        case (DR16_SWITCH_MIDDLE):
        {
            *Switch = DR16_Switch_Status_TRIG_UP_MIDDLE;

            break;
        }
        }

        break;
    }
    case (DR16_SWITCH_DOWN):
    {
        switch (Status)
        {
        case (DR16_SWITCH_UP):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;

            break;
        }
        case (DR16_SWITCH_DOWN):
        {
            *Switch = DR16_Switch_Status_DOWN;

            break;
        }
        case (DR16_SWITCH_MIDDLE):
        {
            *Switch = DR16_Switch_Status_TRIG_DOWN_MIDDLE;

            break;
        }
        }

        break;
    }
    case (DR16_SWITCH_MIDDLE):
    {
        switch (Status)
        {
        case (DR16_SWITCH_UP):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;

            break;
        }
        case (DR16_SWITCH_DOWN):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;

            break;
        }
        case (DR16_SWITCH_MIDDLE):
        {
            *Switch = DR16_Switch_Status_MIDDLE;

            break;
        }
        }

        break;
    }
    }
}

/**
 * @brief 判断按键状态
 *
 */
void Class_DR16::_Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status)
{
    // 带触发的判断
    switch (Pre_Status)
    {
    case (DR16_KEY_FREE):
    {
        switch (Status)
        {
        case (DR16_KEY_FREE):
        {
            *Key = DR16_Key_Status_FREE;

            break;
        }
        case (DR16_KEY_PRESSED):
        {
            *Key = DR16_Key_Status_TRIG_FREE_PRESSED;

            break;
        }
        }

        break;
    }
    case (DR16_KEY_PRESSED):
    {
        switch (Status)
        {
        case (DR16_KEY_FREE):
        {
            *Key = DR16_Key_Status_TRIG_PRESSED_FREE;

            break;
        }
        case (DR16_KEY_PRESSED):
        {
            *Key = DR16_Key_Status_PRESSED;

            break;
        }
        }

        break;
    }
    }
}
