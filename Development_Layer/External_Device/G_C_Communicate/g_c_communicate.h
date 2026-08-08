//
// Created by 谭恩泽 on 2026/4/7.
//

#ifndef C_BOARD_G_C_COMMUNICATE_H
#define C_BOARD_G_C_COMMUNICATE_H

#include "stm32h7xx_hal.h"
#include "drv_fdcan.h"
#include <cstring>

/**
 * @brief 通信状态枚举
 * 
 */
enum Enum_G_C_Status
{
    G_C_Status_DISABLE = 0,
    G_C_Status_ENABLE,
};


//底盘→云台: 接收底盘发送的数据(对应Chassis的SendPacket)
struct G_C_ReceivePacket
{
 uint8_t  Enable : 1;  
 uint8_t  Robot_Level : 3; 
 uint32_t Self_Booster_Heat_Max : 16; //云台热量上限
 uint32_t Self_Booster_Heat_CD : 16;  //云台热量冷却CD
 uint32_t Booster_17mm_1_Heat : 16;   //云台17mm热量
  uint32_t Booster_17mm_Speed : 16;    //云台17mm弹丸速度
  float Chassis_Omega;         //底盘旋转速度
}__attribute__((packed));



//云台→底盘: 发送给底盘的数据(对应Chassis的ReceivePacket)
struct G_C_SendPacket
{
   float target_v_x;
   float target_v_y;
   float target_v_omega;
   uint8_t is_gyro : 1;
   
}__attribute__((packed));


//从原始数据包解码数据
struct Packet_Data
{
     float Self_Booster_Heat_Max;
     float Self_Booster_Heat_CD;
    float Booster_17mm_1_Heat;
    float Booster_17mm_Speed;
    float Chassis_Omega;
};

class G_C_Communicate
{
public:
    void Init(FDCAN_HandleTypeDef *hfdcan);
    //这里的状态是裁判系统在线 & 板间通信使能
    inline bool Get_Status();
    void CAN_RxCpltCallback(Struct_FDCAN_Rx_Buffer *Rx_Buffer);
    void RTOS_100ms_Alive_Callback();
    void RTOS_1ms_Calculate_Callback();
    //底盘计算周期是0.002s，云台计算周期是0.001s，通信周期需要合理
     G_C_SendPacket SendPacket;
     G_C_ReceivePacket ReceivePacket;
     Packet_Data Data;
  
protected:
    void Data_Process(Struct_FDCAN_Rx_Buffer *Rx_Buffer);
     Enum_G_C_Status G_C_Status = G_C_Status_DISABLE;
    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;
    FDCAN_HandleTypeDef *hfdcan_ptr;
};


/**
 * @brief 这里的状态是裁判系统在线 & 板间通信使能
 * 
 * @return true 
 * @return false 
 */
inline bool G_C_Communicate::Get_Status()
{
    bool communication_enbale = (this->G_C_Status == G_C_Status_ENABLE) ? true : false;
    return (communication_enbale & (this->ReceivePacket.Enable == 1));
}

#endif //C_BOARD_G_C_COMMUNICATE_H
