//
// Created by 谭恩泽 on 2025/10/24.
//

#ifndef C_BOARD_LED_H
#define C_BOARD_LED_H

#include "stm32h7xx_hal.h"
#include "tim.h"
#include "gpio.h"


#define Bsp_Buzzer_ON      (1<<3)

/**
 * @brief 板上LED工作状态
 *
 */
enum Enum_BSP_LED_Status
{
    BSP_LED_Status_DISABLED = 0,
    BSP_LED_Status_ENABLED,
};

/**
 * @brief 板载按键状态
 *
 */
enum Enum_BSP_Key_Status
{
    BSP_Key_Status_FREE = 0,
    BSP_Key_Status_PRESSED,
    BSP_Key_Status_TRIG_FREE_TO_PRESSED,
    BSP_Key_Status_TRIG_PRESSED_TO_FREE,
};

void BSP_Init(float Buzzer_Rate = 0);

Enum_BSP_LED_Status BSP_Get_Red_LED();

Enum_BSP_LED_Status BSP_Get_Green_LED();

Enum_BSP_LED_Status BSP_Get_Blue_LED();

Enum_BSP_Key_Status BSP_Get_Key();

void BSP_Set_Red_LED(Enum_BSP_LED_Status Status);

void BSP_Set_Green_LED(Enum_BSP_LED_Status Status);

void BSP_Set_Blue_LED(Enum_BSP_LED_Status Status);

void BSP_Set_PWM_Buzzer(float Rate);

#endif //C_BOARD_LED_H
