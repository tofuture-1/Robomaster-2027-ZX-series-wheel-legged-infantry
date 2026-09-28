//
// Created by 24158 on 2026/6/17.
//

#ifndef RC_ROBOTDOG_REBUILD20260605_FS_I6X_H
#define RC_ROBOTDOG_REBUILD20260605_FS_I6X_H
#include <stdint.h>
#include "drv_uart.h"

#define FS_IA6B_FRAME_LENGTH 32

enum Enum_FS_IA6B_Status {
    FS_IA6B_Status_DISABLE = 0,
    FS_IA6B_Status_ENABLE,
};

/**
 * @brife FS_IA6B接收到的源数据
 *
 */
struct Ibus_msg{
    int16_t	ch[18];
};

class Class_FS_IA6B
{
    public:

    void Init(UART_HandleTypeDef *huart);



    inline int16_t Get_Ibus_msg(int8_t i);//获取接收机回传后被储存的源数据TODO：待进行封装,最好不要在后续直接读取通道值用以调用
    inline Enum_FS_IA6B_Status Get_Status();

    void UART_RxCpltCallback(uint8_t *Rx_Data,uint16_t Length);
    void RTOS_100ms_Alive_Callback();

    protected:
    Struct_UART_Manage_Object *UART_Manage_Object = nullptr;


    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;
    Enum_FS_IA6B_Status FS_IA6B_Status = FS_IA6B_Status_DISABLE;

    Ibus_msg FS_IA6B_ibus_msg;

    bool Data_Process(uint8_t *Rx_Frame);
};

inline int16_t Class_FS_IA6B::Get_Ibus_msg(int8_t i){return FS_IA6B_ibus_msg.ch[i];}
inline Enum_FS_IA6B_Status Class_FS_IA6B::Get_Status(){return FS_IA6B_Status;}
#endif //RC_ROBOTDOG_REBUILD20260605_FS_I6X_H
