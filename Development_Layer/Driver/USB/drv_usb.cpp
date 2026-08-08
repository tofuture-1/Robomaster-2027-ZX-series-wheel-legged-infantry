/**
 * @file    drv_usb.cpp
 * @brief   USB VCP 驱动实现文件
 * @note    非 DMA 模式，适用于对实时性要求不高的场景
 */

#include "drv_usb.h"
#include "usb_device.h"

/** 全局 USB VCP 句柄实例 */
USB_VCP_HandleTypeDef gUsb = {0};

/**
 * @brief  初始化 USB VCP
 * @param  callback: 接收数据回调函数
 * @note   配置接收缓冲区并启动首次接收
 */
void USB_VCP_Init(USB_RxCallback callback) {
    gUsb.rxIdx = 0;
    gUsb.txBusy = 0;
    gUsb.connected = 0;
    gUsb.rxCallback = callback;
    
    // 设置接收缓冲区并启动接收
    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, gUsb.rxBuf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);
}

/**
 * @brief  发送数据到 USB 主机
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval 1: 发送成功，0: 发送失败
 * @note   包含 6 层保护机制：
 *         1. 参数检查 (空指针/长度)
 *         2. 发送忙检查
 *         3. USB 设备状态检查
 *         4. CDC 层状态检查
 *         5. 发送结果检查
 *         6. 状态标志管理
 */
uint8_t USB_VCP_Send(const uint8_t *data, uint16_t len) {
    if (!data || len == 0 || len > USB_TX_BUF_SIZE) {
        return 0;
    }
    
    
    // 3. USB 设备状态检查 - 确保已配置
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) {
        return 0;
    }
    
    // 4. CDC 层状态检查 - 确保 CDC 就绪
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    if (!hcdc || hcdc->TxState) {
        return 0;
    }
    
    // 5. 数据拷贝并发送
    memcpy(gUsb.txBuf, data, len);
    USBD_CDC_SetTxBuffer(&hUsbDeviceHS, gUsb.txBuf, len);
    
    if (USBD_CDC_TransmitPacket(&hUsbDeviceHS) != USBD_OK) {
        return 0;
    }
    
    // 6. 设置发送忙标志
    gUsb.txBusy = 1;
    return 1;
}

/**
 * @brief  获取当前接收缓冲区指针
 * @retval 接收缓冲区指针
 * @note   用于直接访问接收缓冲区数据
 */
uint8_t* USB_VCP_GetRxBuf(void) {
    return gUsb.rxBuf[gUsb.rxIdx];
}

/**
 * @brief  USB 接收数据处理
 * @param  len: 接收数据长度
 * @note   使用双缓冲机制，确保数据不丢失
 *         在中断中执行，回调函数应快速处理
 */
void USB_VCP_RxHandler(uint32_t len) {
    // 无效数据检查
    if (len == 0 || len > USB_RX_BUF_SIZE) {
        USBD_CDC_SetRxBuffer(&hUsbDeviceHS, gUsb.rxBuf[gUsb.rxIdx]);
        USBD_CDC_ReceivePacket(&hUsbDeviceHS);
        return;
    }
    
    // 切换缓冲区索引 (循环使用双缓冲)
    uint8_t bufIdx = gUsb.rxIdx;
    gUsb.rxIdx = (gUsb.rxIdx + 1) % USB_RX_BUF_NUM;
    
    // 设置新的接收缓冲区并启动下一次接收
    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, gUsb.rxBuf[gUsb.rxIdx]);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);
    
    if (gUsb.rxCallback) {
        gUsb.rxCallback(gUsb.rxBuf[bufIdx], (uint16_t)len);
    }
}

/**
 * @brief  USB 发送完成回调
 * @note   在中断中调用，清除发送忙标志
 */
void USB_VCP_TxComplete(void) {
    gUsb.txBusy = 0;
}

/**
 * @brief  设置 USB 连接状态
 */
void USB_VCP_SetConnected(uint8_t state) {
    gUsb.connected = state;
}
