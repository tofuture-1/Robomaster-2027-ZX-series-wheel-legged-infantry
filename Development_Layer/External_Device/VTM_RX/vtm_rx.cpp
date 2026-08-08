#include "vtm_rx.h"

static Class_VTM_RX *Global_VTM_RX_Instance = nullptr;

static void VTM_RX_Bridge_Callback(uint8_t *Buffer, uint16_t Length)
{
    if (Global_VTM_RX_Instance != nullptr)
    {
        Global_VTM_RX_Instance->UART_RxCpltCallback(Buffer, Length);
    }
}


void Class_VTM_RX::Init(UART_HandleTypeDef *huart)
{
    Global_VTM_RX_Instance = this;
    UART_Manage_Object = UART_Init(huart, VTM_RX_Bridge_Callback, VTM_RX_FRAME_LENGTH);
}

void Class_VTM_RX::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    if (Find_And_Store_Frame(Rx_Data, Length) == false)
    {
        return;
    }

    Flag += 1;
    Data_Process(Rx_Data, Length);
}


void Class_VTM_RX::RTOS_100ms_Alive_Callback()
{
    if (Flag == Pre_Flag)
    {
        VTM_RX_Status = VTM_RX_Status_DISABLE;

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
        VTM_RX_Status = VTM_RX_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

void Class_VTM_RX::RTOS_1ms_Calculate_Callback()
{

    const Struct_VTM_RX_UART_Data *tmp_buffer = (Struct_VTM_RX_UART_Data *) UART_Manage_Object->Rx_Buffer[(UART_Manage_Object->Rx_Slot_Index+1)%2];

    _Judge_Key(&Data.Pause_Key, tmp_buffer->Pause_Key, Pre_UART_Rx_Data.Pause_Key);
    _Judge_Key(&Data.Fn_1_Key, tmp_buffer->Fn_1_Key, Pre_UART_Rx_Data.Fn_1_Key);
    _Judge_Key(&Data.Fn_2_Key, tmp_buffer->Fn_2_Key, Pre_UART_Rx_Data.Fn_2_Key);
    _Judge_Key(&Data.Trigger_Key, tmp_buffer->Trigger_Key, Pre_UART_Rx_Data.Trigger_Key);
    _Judge_Key(&Data.Mouse_Left_Key, tmp_buffer->Mouse_Left_Key != 0, Pre_UART_Rx_Data.Mouse_Left_Key != 0);
    _Judge_Key(&Data.Mouse_Right_Key, tmp_buffer->Mouse_Right_Key != 0, Pre_UART_Rx_Data.Mouse_Right_Key != 0);
    _Judge_Key(&Data.Mouse_Middle_Key, tmp_buffer->Mouse_Middle_Key != 0, Pre_UART_Rx_Data.Mouse_Middle_Key != 0);

    for (uint8_t i = 0; i < 16; i++)
    {
        _Judge_Key(&Data.Keyboard_Key[i],
                   (tmp_buffer->Keyboard_Key >> i) & 0x1,
                   (Pre_UART_Rx_Data.Keyboard_Key >> i) & 0x1);
    }

    memcpy(&Pre_UART_Rx_Data, tmp_buffer, sizeof(Struct_VTM_RX_UART_Data));
}

bool Class_VTM_RX::Find_And_Store_Frame(uint8_t *Rx_Data, uint16_t Length)
{
    if (Rx_Data == nullptr || Length < VTM_RX_FRAME_LENGTH)
    {
        return (false);
    }
    if (Rx_Data[0] != VTM_RX_FRAME_SOF_1 || Rx_Data[1] != VTM_RX_FRAME_SOF_2)
    {
        return (false);
    }
    if (!ProtocolCRC::VerifyCRC16CheckSum(Rx_Data, VTM_RX_FRAME_LENGTH))
    {
        return (false);
    }
    return (true);
}

void Class_VTM_RX::Data_Process(uint8_t *Rx_Frame, uint16_t Length)
{
    if (Rx_Frame == nullptr || Length < VTM_RX_FRAME_LENGTH)
    {
        return;
    }
    Struct_VTM_RX_UART_Data *tmp_buffer = (Struct_VTM_RX_UART_Data *) Rx_Frame;

    Data.Channel_0 = ((float)tmp_buffer->Channel_0 - Rocker_Offset) / Rocker_Num;
    Data.Channel_1 = ((float)tmp_buffer->Channel_1 - Rocker_Offset) / Rocker_Num;
    Data.Channel_2 = ((float)tmp_buffer->Channel_2 - Rocker_Offset) / Rocker_Num;
    Data.Channel_3 = ((float)tmp_buffer->Channel_3 - Rocker_Offset) / Rocker_Num;
    Data.Wheel = ((float)tmp_buffer->Wheel - Rocker_Offset) / Rocker_Num;
    Data.Mode_Switch = Convert_Mode_Switch(tmp_buffer->Mode_Switch);
    Data.Mouse_X = ((float)tmp_buffer->Mouse_X) / 32768.0f;
    Data.Mouse_Y = ((float)tmp_buffer->Mouse_Y) / 32768.0f;
    Data.Mouse_Z = ((float)tmp_buffer->Mouse_Z) / 32768.0f;
}

void Class_VTM_RX::_Judge_Key(Enum_VTM_RX_Key_Status *Key, uint8_t Status, uint8_t Pre_Status)
{
    if (Key == nullptr)
    {
        return;
    }

    switch (Pre_Status)
    {
    case 0x00:
    {
        switch (Status)
        {
        case 0x00:
        {
            *Key = VTM_RX_Key_Status_FREE;
            break;
        }
        case 0x01:
        {
            *Key = VTM_RX_Key_Status_TRIG_FREE_PRESSED;
            break;
        }
        }

        break;
    }
    case 0x01:
    {
        switch (Status)
        {
        case 0x00:
        {
            *Key = VTM_RX_Key_Status_TRIG_PRESSED_FREE;
            break;
        }
        case 0x01:
        {
            *Key = VTM_RX_Key_Status_PRESSED;
            break;
        }
        }

        break;
    }
    }
}

Enum_VTM_RX_Mode_Switch Class_VTM_RX::Convert_Mode_Switch(uint8_t Mode_Switch)
{
    switch (Mode_Switch)
    {
    case 0:
    {
        return (VTM_RX_Mode_Switch_C);
    }
    case 1:
    {
        return (VTM_RX_Mode_Switch_N);
    }
    case 2:
    {
        return (VTM_RX_Mode_Switch_S);
    }
    default:
    {
        return (VTM_RX_Mode_Switch_C);
    }
    }
}
