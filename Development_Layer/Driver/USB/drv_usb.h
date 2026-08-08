/**
 * @file    drv_usb.h
 * @brief   USB 虚拟串口 (CDC) 驱动程序 - 非 DMA 模式
 * @note    提供简洁高效的 USB 通信接口，适用于全速/高速 USB 设备
 */

#ifndef DRV_USB_H
#define DRV_USB_H

#include "stm32h7xx_hal.h"
#include "usbd_cdc.h"

/** USB 设备句柄外部声明 */
extern USBD_HandleTypeDef hUsbDeviceHS;

/** 接收缓冲区大小 (字节) - 建议值：256 */
#define USB_RX_BUF_SIZE      256

/** 接收缓冲区数量 - 双缓冲机制 */
#define USB_RX_BUF_NUM       2

/** 发送缓冲区大小 (字节) - 建议值：256 */
#define USB_TX_BUF_SIZE      256

/** USB 接收回调函数类型定义 */
typedef void (*USB_RxCallback)(uint8_t *data, uint16_t len);

/**
 * @brief USB VCP 句柄结构体
 * @note 管理 USB 通信的所有状态和缓冲区
 */
typedef struct {
    uint8_t rxBuf[USB_RX_BUF_NUM][USB_RX_BUF_SIZE];  ///< 双接收缓冲区
    uint8_t txBuf[USB_TX_BUF_SIZE];                   ///< 发送缓冲区
    uint8_t rxIdx;                                    ///< 当前接收缓冲区索引
    volatile uint8_t txBusy;                          ///< 发送忙标志
    volatile uint8_t connected;                       ///< 连接状态标志
    USB_RxCallback rxCallback;                        ///< 接收回调函数指针
} USB_VCP_HandleTypeDef;

/** 全局 USB VCP 句柄 */
extern USB_VCP_HandleTypeDef gUsb;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 USB VCP
 * @param callback: 接收数据回调函数指针
 * @note 此函数应在 USB 设备初始化后调用
 */
void USB_VCP_Init(USB_RxCallback callback);

/**
 * @brief 发送数据到 USB 主机
 * @param data: 发送数据指针
 * @param len:  发送数据长度 (最大 USB_TX_BUF_SIZE)
 * @retval 1: 发送成功，0: 发送失败
 * @note 发送前会自动检查设备状态和发送忙标志
 */
uint8_t USB_VCP_Send(const uint8_t *data, uint16_t len);

/**
 * @brief 获取当前接收缓冲区指针
 * @retval 接收缓冲区指针
 * @note 用于直接访问接收缓冲区
 */
uint8_t* USB_VCP_GetRxBuf(void);

/**
 * @brief USB 接收数据处理 (在中断中调用)
 * @param len: 接收数据长度
 * @note 此函数由 usbd_cdc_if.c 中的 CDC_Receive_HS 调用
 */
void USB_VCP_RxHandler(uint32_t len);

/**
 * @brief USB 发送完成回调 (在中断中调用)
 * @note 此函数由 usbd_cdc_if.c 中的 CDC_TransmitCplt_HS 调用
 */
void USB_VCP_TxComplete(void);

/**
 * @brief 设置 USB 连接状态
 * @param state: 连接状态 (1: 已连接，0: 未连接)
 * @note 此函数由 CDC_Init_HS 和 CDC_DeInit_HS 调用
 */
void USB_VCP_SetConnected(uint8_t state);

/** 调试计数器 */
extern volatile uint32_t usb_send_fail_param;
extern volatile uint32_t usb_send_fail_state;
extern volatile uint32_t usb_send_fail_hcdc;
extern volatile uint32_t usb_send_fail_txstate;
extern volatile uint32_t usb_send_success;

#ifdef __cplusplus
}
#endif

#endif
