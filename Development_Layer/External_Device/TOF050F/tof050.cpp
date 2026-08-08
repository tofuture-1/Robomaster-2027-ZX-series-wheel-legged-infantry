#include "tof050.h"
#include "stm32h7xx_hal.h"

extern Struct_UART_Manage_Object* UART_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function, uint16_t Max_Rx_Length);
extern void UART_Reinit(UART_HandleTypeDef *huart);

Class_TOF050F* Global_TOF050F_Instance = nullptr;

static void TOF050F_Bridge_Callback(uint8_t *buf, uint16_t len);

void Class_TOF050F::Init(UART_HandleTypeDef *huart,
                         uint8_t slave_id,
                         Enum_TOF050F_Range_Mode range_mode,
                         uint16_t poll_period_ms,
                         uint16_t start_delay_ms)
{
    Global_TOF050F_Instance = this;
    UART_Handler = huart;
    Slave_ID = slave_id;
    Range_Mode = range_mode;
    Poll_Period_ms = (poll_period_ms < 50) ? 50 : poll_period_ms;
    Poll_Counter_ms = 0;
    Start_Delay_ms = start_delay_ms;

    if (huart != nullptr)
    {
        UART_Manage_Object = UART_Init(huart, TOF050F_Bridge_Callback, 64);
    }
    else
    {
        UART_Manage_Object = nullptr;
    }

    Flag = 0;
    Pre_Flag = 0;
    Offline_Count_100ms = 0;

    Set_Range_Mode(range_mode);

    Status = TOF050F_Status_DISABLE;
    Distance_MM = TOF050F_DISTANCE_INVALID_MM;
    Distance_M = 0.0f;
    Last_Update_Timestamp = 0;

    Rx_OK_Cnt = 0;
    CRC_Fail_Cnt = 0;
    Write_ACK_Cnt = 0;
    std::memset(Tx_Buffer, 0, sizeof(Tx_Buffer));
}

uint16_t Class_TOF050F::Get_Distance_MM()
{
    if (Distance_MM < Distance_Min_MM || Distance_MM > Distance_Max_MM)
    {
        return Last_Valid_Distance_MM;
    }

    if (Last_Valid_Distance_MM != TOF050F_DISTANCE_INVALID_MM)
    {
        int16_t diff = static_cast<int16_t>(Distance_MM) - static_cast<int16_t>(Last_Valid_Distance_MM);
        if (diff > static_cast<int16_t>(Distance_Jump_Threshold_MM) || diff < -static_cast<int16_t>(Distance_Jump_Threshold_MM))
        {
            return Last_Valid_Distance_MM;
        }
    }

    Last_Valid_Distance_MM = Distance_MM;

    if (Distance_MM >= 1000) {
        return 1000;
    }
    return Distance_MM;
}

void Class_TOF050F::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    if (Rx_Data == nullptr || Length == 0)
    {
        return;
    }

    Data_Process(Rx_Data, Length);
}

static void TOF050F_Bridge_Callback(uint8_t *buf, uint16_t len)
{
    extern Class_TOF050F* Global_TOF050F_Instance;
    if (Global_TOF050F_Instance != nullptr)
    {
        Global_TOF050F_Instance->UART_RxCpltCallback(buf, len);
    }
}

void Class_TOF050F::TIM_1ms_Poll_PeriodElapsedCallback()
{
    if (UART_Handler == nullptr)
    {
        return;
    }

    if (Start_Delay_ms > 0)
    {
        Start_Delay_ms--;
        return;
    }

    Poll_Counter_ms++;
    if (Poll_Counter_ms >= Poll_Period_ms)
    {
        Poll_Counter_ms = 0;
        Send_Read_Distance();
    }
}

void Class_TOF050F::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        if (Offline_Count_100ms < 255)
        {
            Offline_Count_100ms++;
        }

        if (Offline_Count_100ms >= 5)
        {
            Status = TOF050F_Status_DISABLE;
            Distance_MM = TOF050F_DISTANCE_INVALID_MM;
            Distance_M = 0.0f;

            if (UART_Handler != nullptr)
            {
                UART_Reinit(UART_Handler);
            }
        }
    }
    else
    {
        Offline_Count_100ms = 0;
        Status = TOF050F_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

void Class_TOF050F::Send_Read_Distance()
{
    if (UART_Handler == nullptr)
    {
        return;
    }

    Fill_Modbus_Frame(0x03, TOF050F_READ_DISTANCE_REG, 0x0001);
    (void)UART_Send_Data(UART_Handler, Tx_Buffer, sizeof(Tx_Buffer));
}

void Class_TOF050F::Set_Range_Mode(Enum_TOF050F_Range_Mode range_mode)
{
    Range_Mode = range_mode;
    Send_Write_Register(TOF050F_RANGE_MODE_REG, static_cast<uint16_t>(range_mode));
}

void Class_TOF050F::Set_Auto_Output(uint16_t period_ms)
{
    Send_Write_Register(TOF050F_AUTO_OUTPUT_REG, period_ms);
}

uint16_t Class_TOF050F::CRC16_Modbus(const uint8_t *data, uint16_t length) const
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if ((crc & 0x0001) != 0)
            {
                crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
            }
            else
            {
                crc = static_cast<uint16_t>(crc >> 1);
            }
        }
    }

    return crc;
}

bool Class_TOF050F::Verify_CRC16_Modbus(const uint8_t *frame, uint16_t length) const
{
    if (frame == nullptr || length < 3)
    {
        return false;
    }

    uint16_t crc_calc = CRC16_Modbus(frame, static_cast<uint16_t>(length - 2));
    uint16_t crc_recv = static_cast<uint16_t>(frame[length - 2]) |
                        static_cast<uint16_t>(frame[length - 1] << 8);

    return (crc_calc == crc_recv);
}

void Class_TOF050F::Fill_Modbus_Frame(uint8_t function_code, uint16_t reg_addr, uint16_t value)
{
    Tx_Buffer[0] = Slave_ID;
    Tx_Buffer[1] = function_code;
    Tx_Buffer[2] = static_cast<uint8_t>((reg_addr >> 8) & 0xFF);
    Tx_Buffer[3] = static_cast<uint8_t>(reg_addr & 0xFF);
    Tx_Buffer[4] = static_cast<uint8_t>((value >> 8) & 0xFF);
    Tx_Buffer[5] = static_cast<uint8_t>(value & 0xFF);

    uint16_t crc = CRC16_Modbus(Tx_Buffer, 6);
    Tx_Buffer[6] = static_cast<uint8_t>(crc & 0xFF);
    Tx_Buffer[7] = static_cast<uint8_t>((crc >> 8) & 0xFF);
}

void Class_TOF050F::Send_Write_Register(uint16_t reg_addr, uint16_t value)
{
    if (UART_Handler == nullptr)
    {
        return;
    }

    Fill_Modbus_Frame(0x06, reg_addr, value);
    (void)UART_Send_Data(UART_Handler, Tx_Buffer, sizeof(Tx_Buffer));
}

void Class_TOF050F::Data_Process(uint8_t *Rx_Data, uint16_t Length)
{
    if (Rx_Data == nullptr || Length < 7)
    {
        return;
    }

    for (uint16_t i = 0; (i + 1) < Length; i++)
    {
        if (Rx_Data[i] != Slave_ID)
        {
            continue;
        }

        uint8_t function_code = Rx_Data[i + 1];

        if (function_code == 0x03)
        {
            if ((i + 7) > Length)
            {
                break;
            }

            uint8_t byte_count = Rx_Data[i + 2];
            uint16_t frame_len = static_cast<uint16_t>(byte_count + 5);

            if (byte_count != 0x02 || (i + frame_len) > Length)
            {
                continue;
            }

            if (!Verify_CRC16_Modbus(&Rx_Data[i], frame_len))
            {
                CRC_Fail_Cnt++;
                continue;
            }

            uint16_t distance = static_cast<uint16_t>((Rx_Data[i + 3] << 8) | Rx_Data[i + 4]);

            Distance_MM = distance;
            Distance_M = static_cast<float>(Distance_MM) * 0.001f;
            Last_Update_Timestamp = HAL_GetTick();
            Rx_OK_Cnt++;
            Flag++;
            Status = TOF050F_Status_ENABLE;

            i = static_cast<uint16_t>(i + frame_len - 1);
        }
        else if (function_code == 0x06)
        {
            if ((i + 8) > Length)
            {
                break;
            }

            if (!Verify_CRC16_Modbus(&Rx_Data[i], 8))
            {
                CRC_Fail_Cnt++;
                continue;
            }

            Write_ACK_Cnt++;
            i = static_cast<uint16_t>(i + 7);
        }
    }
}
