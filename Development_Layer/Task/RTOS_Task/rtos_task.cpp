//
// Created by 谭恩泽 on 2025/10/30.
//

#include "rtos_task.h"

/*添加cpp头文件*/
#include "callback_task.h"
#include "fdcan.h"
#include "drv_fdcan.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "drv_spi.h"
#include "led_and_buzzer.h"
#include "spi.h"
#include "usart.h"
#include "dc_source.h"

void Robot_Init_Task()
{
    // 驱动层初始化

    // 点俩灯,
    BSP_Init(0.0f); //c板蜂鸣器在0.1-0.9能响，

    //定时器初始化
    TIM_Init(&htim6, TIM6_Robot_1ms_Callback); //1ms定时器
    // CAN总线初始化
    FDCAN_Init(&hfdcan1, Device_FDCAN1_Callback);
    FDCAN_Init(&hfdcan2, Device_FDCAN2_Callback);
    FDCAN_Init(&hfdcan3, Device_FDCAN3_Callback);
    // UART初始化
    UART_Init(&huart7, FSIA6B_RX_Callback, 32);
    //UART_Init(&huart7, USART2_RX_Callback, 32);
    //SPI初始化
    SPI_Init(&hspi2, BMI088_SPI2_Callback);

    //开启5V,24V输出
    __5V_DC_Power_On();
    __24V_DC_Power_On();  
    //UART_Init(&huart6, VOFA_UART6_Callback, 8);

   // SPI_Init(&hspi2, Board_IMU_SPI_Callback);

    // 设备层初始化

    // 串口绘图初始化

    // 战车层初始化

    // 交互层初始化

    //机器人战车初始化
    robot.Init();

    // 使能定时器时基单元和定时器中断
    HAL_TIM_Base_Start_IT(&htim6);
    // 标记初始化完成
    init_finished = true;
		
    // 等待系统
    HAL_Delay(1000);

}




//主线程
void Robot_Main_Loop()
{
    robot.Main_Loop_Things();
}



