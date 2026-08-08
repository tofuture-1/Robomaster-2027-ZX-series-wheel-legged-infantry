#ifndef __VTM_RX_H__
#define __VTM_RX_H__

#include "protocol_crc.h"
#include "drv_uart.h"
#include "string.h"

#define VTM_RX_FRAME_SOF_1 (0xA9)
#define VTM_RX_FRAME_SOF_2 (0x53)
#define VTM_RX_FRAME_LENGTH (21)

#define VTM_RX_KEY_W (0)
#define VTM_RX_KEY_S (1)
#define VTM_RX_KEY_A (2)
#define VTM_RX_KEY_D (3)
#define VTM_RX_KEY_SHIFT (4)
#define VTM_RX_KEY_CTRL (5)
#define VTM_RX_KEY_Q (6)
#define VTM_RX_KEY_E (7)
#define VTM_RX_KEY_R (8)
#define VTM_RX_KEY_F (9)
#define VTM_RX_KEY_G (10)
#define VTM_RX_KEY_Z (11)
#define VTM_RX_KEY_X (12)
#define VTM_RX_KEY_C (13)
#define VTM_RX_KEY_V (14)
#define VTM_RX_KEY_B (15)

enum Enum_VTM_RX_Status {
    VTM_RX_Status_DISABLE = 0,
    VTM_RX_Status_ENABLE,
};

enum Enum_VTM_RX_Key_Status {
    VTM_RX_Key_Status_FREE = 0,
    VTM_RX_Key_Status_PRESSED,
    VTM_RX_Key_Status_TRIG_FREE_PRESSED,
    VTM_RX_Key_Status_TRIG_PRESSED_FREE,
};

enum Enum_VTM_RX_Mode_Switch : uint8_t {
    VTM_RX_Mode_Switch_C = 0,
    VTM_RX_Mode_Switch_N,
    VTM_RX_Mode_Switch_S,
};

struct Struct_VTM_RX_UART_Data {
    uint8_t SOF_1;
    uint8_t SOF_2;
    uint32_t Channel_0 : 11;
    uint32_t Channel_1 : 11;
    uint32_t Channel_2 : 11;
    uint32_t Channel_3 : 11;
    uint32_t Mode_Switch : 2;
    uint32_t Pause_Key : 1;
    uint32_t Fn_1_Key : 1;
    uint32_t Fn_2_Key : 1;
    uint32_t Wheel : 11;
    uint32_t Trigger_Key : 1;
    int16_t Mouse_X;
    int16_t Mouse_Y;
    int16_t Mouse_Z;
    uint8_t Mouse_Left_Key : 2;
    uint8_t Mouse_Right_Key : 2;
    uint8_t Mouse_Middle_Key : 2;
    uint16_t Keyboard_Key;
    uint16_t CRC16;
} __attribute__((packed));

struct Struct_VTM_RX_Data {
    float Channel_0 = 0.0f;
    float Channel_1 = 0.0f;
    float Channel_2 = 0.0f;
    float Channel_3 = 0.0f;
    float Wheel = 0.0f;
    Enum_VTM_RX_Mode_Switch Mode_Switch = VTM_RX_Mode_Switch_C;
    float Mouse_X = 0.0f;
    float Mouse_Y = 0.0f;
    float Mouse_Z = 0.0f;
    Enum_VTM_RX_Key_Status Pause_Key = VTM_RX_Key_Status_FREE;
    Enum_VTM_RX_Key_Status Fn_1_Key = VTM_RX_Key_Status_FREE;
    Enum_VTM_RX_Key_Status Fn_2_Key = VTM_RX_Key_Status_FREE;
    Enum_VTM_RX_Key_Status Trigger_Key = VTM_RX_Key_Status_FREE;
    Enum_VTM_RX_Key_Status Mouse_Left_Key = VTM_RX_Key_Status_FREE;
    Enum_VTM_RX_Key_Status Mouse_Right_Key = VTM_RX_Key_Status_FREE;
    Enum_VTM_RX_Key_Status Mouse_Middle_Key = VTM_RX_Key_Status_FREE;
    Enum_VTM_RX_Key_Status Keyboard_Key[16] = {};
};

class Class_VTM_RX {
public:
    void Init(UART_HandleTypeDef *huart);

    inline Enum_VTM_RX_Status Get_Status();
    inline float Get_Right_X();
    inline float Get_Right_Y();
    inline float Get_Left_Y();
    inline float Get_Left_X();
    inline float Get_Wheel();
    inline Enum_VTM_RX_Mode_Switch Get_Mode_Switch();
    inline float Get_Mouse_X();
    inline float Get_Mouse_Y();
    inline float Get_Mouse_Z();
    inline Enum_VTM_RX_Key_Status Get_Pause_Key();
    inline Enum_VTM_RX_Key_Status Get_Fn_1_Key();
    inline Enum_VTM_RX_Key_Status Get_Fn_2_Key();
    inline Enum_VTM_RX_Key_Status Get_Trigger_Key();
    inline Enum_VTM_RX_Key_Status Get_Mouse_Left_Key();
    inline Enum_VTM_RX_Key_Status Get_Mouse_Right_Key();
    inline Enum_VTM_RX_Key_Status Get_Mouse_Middle_Key();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key(uint8_t index);
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_W();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_S();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_A();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_D();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_SHIFT();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_CTRL();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_Q();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_E();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_R();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_F();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_G();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_Z();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_X();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_C();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_V();
    inline Enum_VTM_RX_Key_Status Get_Keyboard_Key_B();

    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);
    void RTOS_100ms_Alive_Callback();
    void RTOS_1ms_Calculate_Callback();

protected:
    Struct_UART_Manage_Object *UART_Manage_Object = nullptr;

    float Rocker_Offset = 1024.0f;
    float Rocker_Num = 660.0f;

    Struct_VTM_RX_UART_Data Pre_UART_Rx_Data = {};
    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    Enum_VTM_RX_Status VTM_RX_Status = VTM_RX_Status_DISABLE;
    Struct_VTM_RX_Data Data = {};

    bool Find_And_Store_Frame(uint8_t *Rx_Data, uint16_t Length);
    void Data_Process(uint8_t *Rx_Frame, uint16_t Length);
    void _Judge_Key(Enum_VTM_RX_Key_Status *Key, uint8_t Status, uint8_t Pre_Status);
    Enum_VTM_RX_Mode_Switch Convert_Mode_Switch(uint8_t Mode_Switch);

};

inline Enum_VTM_RX_Status Class_VTM_RX::Get_Status()
{
    return (VTM_RX_Status);
}

inline float Class_VTM_RX::Get_Right_X()
{
    return (Data.Channel_0);
}

inline float Class_VTM_RX::Get_Right_Y()
{
    return (Data.Channel_1);
}

inline float Class_VTM_RX::Get_Left_Y()
{
    return (Data.Channel_2);
}

inline float Class_VTM_RX::Get_Left_X()
{
    return (Data.Channel_3);
}

inline float Class_VTM_RX::Get_Wheel()
{
    return (Data.Wheel);
}

inline Enum_VTM_RX_Mode_Switch Class_VTM_RX::Get_Mode_Switch()
{
    return (Data.Mode_Switch);
}

inline float Class_VTM_RX::Get_Mouse_X()
{
    return (Data.Mouse_X);
}

inline float Class_VTM_RX::Get_Mouse_Y()
{
    return (Data.Mouse_Y);
}

inline float Class_VTM_RX::Get_Mouse_Z()
{
    return (Data.Mouse_Z);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Pause_Key()
{
    return (Data.Pause_Key);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Fn_1_Key()
{
    return (Data.Fn_1_Key);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Fn_2_Key()
{
    return (Data.Fn_2_Key);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Trigger_Key()
{
    return (Data.Trigger_Key);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Mouse_Left_Key()
{
    return (Data.Mouse_Left_Key);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Mouse_Right_Key()
{
    return (Data.Mouse_Right_Key);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Mouse_Middle_Key()
{
    return (Data.Mouse_Middle_Key);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key(uint8_t index)
{
    if (index >= 16)
    {
        return (VTM_RX_Key_Status_FREE);
    }

    return (Data.Keyboard_Key[index]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_W()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_W]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_S()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_S]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_A()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_A]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_D()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_D]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_SHIFT()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_SHIFT]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_CTRL()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_CTRL]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_Q()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_Q]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_E()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_E]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_R()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_R]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_F()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_F]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_G()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_G]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_Z()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_Z]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_X()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_X]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_C()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_C]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_V()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_V]);
}

inline Enum_VTM_RX_Key_Status Class_VTM_RX::Get_Keyboard_Key_B()
{
    return (Data.Keyboard_Key[VTM_RX_KEY_B]);
}

#endif /* __VTM_RX_H__ */
