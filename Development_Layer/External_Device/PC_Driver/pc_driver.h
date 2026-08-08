#ifndef PC_DRIVER_H
#define PC_DRIVER_H

#include "drv_usb.h"
#include "pc_packet.h"

enum Enum_PC_Status
{
    PC_Status_DISABLE = 0,
    PC_Status_ENABLE,
};

typedef void (*PC_RxCallback)(const PC_ReceivePacket& packet);

class Robot_PC {
public:
    uint8_t origin_data[50];

    void Init(PC_RxCallback cb);

    bool IsConnected() const { 
        return gUsb.connected && (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED); 
    }

    bool IsTxReady() const { 
        return !gUsb.txBusy; 
    }
    
    bool Send(const PC_SendPacket& packet);
    
    void HandleRxData(uint8_t* data, uint16_t len);

    PC_SendPacket txPacket;
    PC_ReceivePacket rxPacket;
    volatile bool rxUpdated = false;

    Enum_PC_Status Get_Status();

    void RTOS_100ms_Alive_Callback();

private:

    PC_RxCallback rxCallback;
    Enum_PC_Status PC_Status = PC_Status_DISABLE;
    uint16_t Offline_Count = 0;
    uint8_t Flag = 0;
    uint8_t Pre_Flag = 0;
};

void PC_USB_RxCallback(uint8_t *data, uint16_t len);

#endif
