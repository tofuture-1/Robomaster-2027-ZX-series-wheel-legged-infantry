/**
* @file Serial_Debug.h
 * @brief 串口调试工具 (支持波形显示 & 日志分级着色)
 */

#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H

#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// 定义最大调试缓冲区大小
#define DEBUG_BUFFER_SIZE 256

class Class_Serial_Debug {
public:
    void Init(UART_HandleTypeDef *huart);

    // --- 基础功能 ---
    // 原始格式化打印 (不带标签)
    void Printf(const char *format, ...);

    // 发送波形数据 (小端浮点数组协议, 适配 serial.xywml.com)
    void Plot(int num, ...);

    // 发送 IMU 欧拉角数据，适配 VOFA+ Cube 控件的欧拉角 X/Y/Z
    // (通道顺序: I0=Roll, I1=Pitch, I2=Yaw)
    void PlotEuler(float roll_deg, float pitch_deg, float yaw_deg);

    // 发送 IMU 四元数数据，适配 VOFA+ Cube 控件的四元数 W/X/Y/Z
    // (通道顺序: I0=q_w, I1=q_x, I2=q_y, I3=q_z)
    void PlotQuaternion(float q_w, float q_x, float q_y, float q_z);

    // --- 新增：分级日志功能 (自动着色) ---
    // 灰色：调试信息
    void Log_Debug(const char *format, ...);
    // 绿色/蓝色：一般信息
    void Log_Info(const char *format, ...);
    // 黄色：警告信息
    void Log_Warn(const char *format, ...);
    // 红色：错误信息
    void Log_Error(const char *format, ...);

private:
    UART_HandleTypeDef *huart_handle;
    uint8_t tx_buffer[2][DEBUG_BUFFER_SIZE];  // 双缓冲
    uint8_t tx_buffer_index;                  // 当前使用的缓冲索引
    uint8_t rx_buffer[1]; // 如果需要接收功能可扩展

    // 内部底层发送函数
    void Send_Raw(uint8_t *data, uint16_t len);

    // 内部辅助函数：处理带标签的日志
    void Print_With_Label(const char *label, const char *format, va_list args);
};


#endif // SERIAL_DEBUG_H
