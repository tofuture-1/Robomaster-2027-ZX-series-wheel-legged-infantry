/**
 * @file Serial_Debug.cpp
 */

#include "Serial_Debug.h"
#include "string.h"


// 协议帧尾 (小端浮点数组协议)
const uint8_t FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};

void Class_Serial_Debug::Init(UART_HandleTypeDef *huart) {
    this->huart_handle = huart;
    this->tx_buffer_index = 0;
}

void Class_Serial_Debug::Send_Raw(uint8_t *data, uint16_t len) {
    if (this->huart_handle == nullptr) return;
    HAL_UART_Transmit_DMA(this->huart_handle, data, len);
}

// -------------------------------------------------------------------------
//  内部核心逻辑：拼接标签 + 格式化内容 + 自动换行
// -------------------------------------------------------------------------
void Class_Serial_Debug::Print_With_Label(const char *label, const char *format, va_list args) {
    // 使用当前缓冲索引
    uint8_t *buffer = this->tx_buffer[this->tx_buffer_index];
    
    // 1. 先把标签拷贝进去 (例如 "[ERROR] ")
    // snprintf 会返回写入的长度
    int offset = snprintf((char *)buffer, DEBUG_BUFFER_SIZE, "%s", label);

    // 2. 接着标签后面，格式化用户的内容
    if (offset < DEBUG_BUFFER_SIZE) {
        int len = vsnprintf((char *)buffer + offset, DEBUG_BUFFER_SIZE - offset, format, args);

        // 计算总长度
        int total_len = offset + len;

        // 3. 自动补回车换行 (如果用户没加)
        if (total_len > 0 && total_len < DEBUG_BUFFER_SIZE - 2) {
            if (buffer[total_len - 1] != '\n') {
                buffer[total_len++] = '\r';
                buffer[total_len++] = '\n';
            }
        }

        // 4. 发送并切换缓冲
        Send_Raw(buffer, (uint16_t)total_len);
        this->tx_buffer_index = 1 - this->tx_buffer_index;  // 切换缓冲: 0->1, 1->0
    }
}

// -------------------------------------------------------------------------
//  分级日志接口实现
// -------------------------------------------------------------------------

void Class_Serial_Debug::Log_Debug(const char *format, ...) {
    va_list args;
    va_start(args, format);
    Print_With_Label("[DEBUG] ", format, args);
    va_end(args);
}

void Class_Serial_Debug::Log_Info(const char *format, ...) {
    va_list args;
    va_start(args, format);
    Print_With_Label("[INFO] ", format, args);
    va_end(args);
}

void Class_Serial_Debug::Log_Warn(const char *format, ...) {
    va_list args;
    va_start(args, format);
    Print_With_Label("[WARN] ", format, args);
    va_end(args);
}

void Class_Serial_Debug::Log_Error(const char *format, ...) {
    va_list args;
    va_start(args, format);
    Print_With_Label("[ERROR] ", format, args);
    va_end(args);
}

// -------------------------------------------------------------------------
//  原有功能保持不变
// -------------------------------------------------------------------------

void Class_Serial_Debug::Printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    // Printf 不带标签，直接用空字符串
    Print_With_Label("", format, args);
    va_end(args);
}

void Class_Serial_Debug::Plot(int num, ...) {
    if (num > 20) num = 20;
    
    // 使用当前缓冲索引
    uint8_t *buffer = this->tx_buffer[this->tx_buffer_index];
    
    va_list args;
    va_start(args, num);
    int offset = 0;
    for (int i = 0; i < num; i++) {
        float val = (float)va_arg(args, double);
        if (offset + sizeof(float) <= DEBUG_BUFFER_SIZE - 4) {
            memcpy(buffer + offset, &val, sizeof(float));
            offset += sizeof(float);
        }
    }
    va_end(args);
    if (offset + 4 <= DEBUG_BUFFER_SIZE) {
        memcpy(buffer + offset, FRAME_TAIL, 4);
        offset += 4;
    }
    Send_Raw(buffer, (uint16_t)offset);
    this->tx_buffer_index = 1 - this->tx_buffer_index;  // 切换缓冲: 0->1, 1->0
}

// -------------------------------------------------------------------------
//  IMU 姿态数据发送（适配 VOFA+ Cube 控件）
//  FireWater 协议，一帧 = 若干小端 float + 帧尾 00 00 80 7F
// -------------------------------------------------------------------------

void Class_Serial_Debug::PlotEuler(float roll_deg, float pitch_deg, float yaw_deg) {
    uint8_t *buffer = this->tx_buffer[this->tx_buffer_index];
    float data[3] = {roll_deg, pitch_deg, yaw_deg};
    memcpy(buffer, data, sizeof(data));          // 3 个 float = 12 字节
    memcpy(buffer + sizeof(data), FRAME_TAIL, 4);
    Send_Raw(buffer, sizeof(data) + 4);
    this->tx_buffer_index = 1 - this->tx_buffer_index;
}

void Class_Serial_Debug::PlotQuaternion(float q_w, float q_x, float q_y, float q_z) {
    uint8_t *buffer = this->tx_buffer[this->tx_buffer_index];
    float data[4] = {q_w, q_x, q_y, q_z};
    memcpy(buffer, data, sizeof(data));          // 4 个 float = 16 字节
    memcpy(buffer + sizeof(data), FRAME_TAIL, 4);
    Send_Raw(buffer, sizeof(data) + 4);
    this->tx_buffer_index = 1 - this->tx_buffer_index;
}
