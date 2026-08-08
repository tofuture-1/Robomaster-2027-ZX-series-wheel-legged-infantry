#ifndef __TOF050_H
#define __TOF050_H

#include <cstdint>
#include <cstring>

#include "drv_uart.h"

#define TOF050F_DEFAULT_SLAVE_ID        ((uint8_t)0x01)
#define TOF050F_READ_DISTANCE_REG       ((uint16_t)0x0010)
#define TOF050F_RANGE_MODE_REG          ((uint16_t)0x0004)
#define TOF050F_AUTO_OUTPUT_REG         ((uint16_t)0x0005)
#define TOF050F_DISTANCE_INVALID_MM     ((uint16_t)0xFFFF)

enum Enum_TOF050F_Status
{
    TOF050F_Status_DISABLE = 0,
    TOF050F_Status_ENABLE,
};

enum Enum_TOF050F_Range_Mode : uint16_t
{
    TOF050F_Range_Mode_HIGH_PRECISION = 0x0001,
    TOF050F_Range_Mode_MIDDLE         = 0x0002,
    TOF050F_Range_Mode_LONG           = 0x0003,
};

class Class_TOF050F
{
public:
    void Init(UART_HandleTypeDef *huart,
              uint8_t slave_id = TOF050F_DEFAULT_SLAVE_ID,
              Enum_TOF050F_Range_Mode range_mode = TOF050F_Range_Mode_LONG,
              uint16_t poll_period_ms = 100,
              uint16_t start_delay_ms = 0);

    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);

    void TIM_1ms_Poll_PeriodElapsedCallback();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void Send_Read_Distance();

    void Set_Range_Mode(Enum_TOF050F_Range_Mode range_mode);

    void Set_Auto_Output(uint16_t period_ms);

    inline Enum_TOF050F_Status Get_Status() const;

    uint16_t Get_Distance_MM();

    inline float Get_Distance_M() const;

    inline uint32_t Get_Rx_OK_Cnt() const;

    inline uint32_t Get_CRC_Fail_Cnt() const;

    inline uint64_t Get_Last_Update_Timestamp() const;

protected:
    UART_HandleTypeDef *UART_Handler = nullptr;
    Struct_UART_Manage_Object *UART_Manage_Object = nullptr;

    uint8_t Slave_ID = TOF050F_DEFAULT_SLAVE_ID;
    Enum_TOF050F_Range_Mode Range_Mode = TOF050F_Range_Mode_LONG;

    uint16_t Poll_Period_ms = 100;
    uint16_t Poll_Counter_ms = 0;
    uint16_t Start_Delay_ms = 0;

    uint8_t Tx_Buffer[8] = {0};

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;
    uint8_t Offline_Count_100ms = 0;

    Enum_TOF050F_Status Status = TOF050F_Status_DISABLE;
    uint16_t Distance_MM = TOF050F_DISTANCE_INVALID_MM;
    uint16_t Last_Valid_Distance_MM = TOF050F_DISTANCE_INVALID_MM;
    uint16_t Distance_Jump_Threshold_MM = 50;
    uint16_t Distance_Min_MM = 55;
    uint16_t Distance_Max_MM = 160;
    float Distance_M = 0.0f;
    uint64_t Last_Update_Timestamp = 0;

    uint32_t Rx_OK_Cnt = 0;
    uint32_t CRC_Fail_Cnt = 0;
    uint32_t Write_ACK_Cnt = 0;

    uint16_t CRC16_Modbus(const uint8_t *data, uint16_t length) const;

    bool Verify_CRC16_Modbus(const uint8_t *frame, uint16_t length) const;

    void Fill_Modbus_Frame(uint8_t function_code, uint16_t reg_addr, uint16_t value);

    void Send_Write_Register(uint16_t reg_addr, uint16_t value);

    void Data_Process(uint8_t *Rx_Data, uint16_t Length);
};

inline Enum_TOF050F_Status Class_TOF050F::Get_Status() const
{
    return Status;
}

inline float Class_TOF050F::Get_Distance_M() const
{
    return Distance_M;
}

inline uint32_t Class_TOF050F::Get_Rx_OK_Cnt() const
{
    return Rx_OK_Cnt;
}

inline uint32_t Class_TOF050F::Get_CRC_Fail_Cnt() const
{
    return CRC_Fail_Cnt;
}

inline uint64_t Class_TOF050F::Get_Last_Update_Timestamp() const
{
    return Last_Update_Timestamp;
}

extern Class_TOF050F* Global_TOF050F_Instance;


#endif
