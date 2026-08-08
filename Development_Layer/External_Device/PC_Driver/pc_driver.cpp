/**
 * @file    pc_driver.cpp
 * @brief   PC 通信驱动实现文件
 * @note    基于 USB 虚拟串口，提供与 PC 端的数据通信
 */

#include "pc_driver.h"
#include "protocol_crc.h"

/** 全局 PC 驱动实例指针 - 用于 C 回调函数访问 */
static Robot_PC *g_pc_instance = nullptr;


/**
 * @brief  USB 接收回调函数 (C 语言接口)
 * @param  data: 接收数据指针
 * @param  len: 接收数据长度
 * @note   由 USB 驱动层调用，转发到 C++ 类处理
 */
void PC_USB_RxCallback(uint8_t *data, uint16_t len) {
    if (g_pc_instance) {
        g_pc_instance->HandleRxData(data, len);
    }
}

/**
 * @brief  初始化 PC 通信
 * @param  cb: 接收数据回调函数
 * @note   设置全局实例指针并初始化 USB 驱动
 */
void Robot_PC::Init(PC_RxCallback cb) {
    g_pc_instance = this;
    rxCallback = cb;
    PC_Status = PC_Status_DISABLE;
    Offline_Count = 0;
    Pre_Flag = 0;
    USB_VCP_Init(PC_USB_RxCallback);
}

/**
 * @brief 获取 PC 在线状态
 * @return Enum_PC_Status PC 在线状态
 * @note 结合 USB 硬件连接状态和数据接收状态综合判断
 */
Enum_PC_Status Robot_PC::Get_Status() {
    // 首先检查 USB 硬件连接状态
    if (!IsConnected())
    {
        return PC_Status_DISABLE;
    }
    return PC_Status;
}

/**
 * @brief 100ms 定时器回调，用于在线检测
 * @note 参考 DR16 实现，每 100ms 调用一次
 *       判断该时间段内是否接收过 PC 数据
 */
void Robot_PC::RTOS_100ms_Alive_Callback() {
    // 判断该时间段内是否接收过 PC 数据
    if (Flag == Pre_Flag)
    {
        Offline_Count++;
        if (Offline_Count >= 2)
        {
            PC_Status = PC_Status_DISABLE;
            Offline_Count = 0;
        }
    }
    else
    {
        PC_Status = PC_Status_ENABLE;
        Offline_Count = 0;
    }
    Pre_Flag = Flag;
}



/**
 * @brief  处理接收到的数据
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @note   数据校验流程:
 *         1. 长度检查 - 确保数据完整
 *         2. 帧头检查 - 验证数据起始 (0x5A)
 *         3. CRC16 校验和检查 - 确保数据正确
 *         4. 回调处理 - 转发到上层应用
 */
void Robot_PC::HandleRxData(uint8_t* data, uint16_t len) {
    // 1. 长度检查 - 数据必须至少包含一个完整数据包
    if (len < sizeof(PC_ReceivePacket)) return;

    // 2. 帧头检查 - 验证起始字节 (0x5A)
    if (data[0] != 'S' || data[1] != 'P') return;

    // 3. CRC16 校验和检查 - 验证数据完整性
    if (!ProtocolCRC::VerifyCRC16CheckSum(data, sizeof(PC_ReceivePacket))) return;

    // 4. 设置接收标志，用于在线检测
    Flag++;

    // 5. 拷贝数据到类成员变量
    memcpy(origin_data, data, sizeof(PC_ReceivePacket));
    memcpy(&rxPacket, data, sizeof(PC_ReceivePacket));
    rxUpdated = true;

    // 6. 回调处理 - 转发到上层应用
    if (rxCallback) rxCallback(rxPacket);
}


bool Robot_PC::Send(const PC_SendPacket& packet) {
    txPacket = packet;
    ProtocolCRC::AppendCRC16CheckSum((uint8_t*)&txPacket, sizeof(PC_SendPacket));
    return USB_VCP_Send((const uint8_t*)&txPacket, sizeof(PC_SendPacket));
}
