//
// Created by 谭恩泽 on 2025/10/24.
//

 #include "led_and_buzzer.h"


/**
 * @brief 初始化全部板级支持包引脚
 *
 * @param Status 各个状态的按位或
 */
void BSP_Init(float Buzzer_Rate)
{
    HAL_TIM_Base_Start(&htim12); // PWM定时器开启
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); // 蜂鸣器PWM通道开启
    BSP_Set_PWM_Buzzer(Buzzer_Rate);
}

///**
// * @brief 获取红色LED
// *
// * @return Enum_BSP_LED_Status 状态
// */
//Enum_BSP_LED_Status BSP_Get_Red_LED()
//{
// 
//}

///**
// * @brief 绿色LED
// *
// * @return Enum_BSP_LED_Status 状态
// */
//Enum_BSP_LED_Status BSP_Get_Green_LED()
//{
//    
//}

///**
// * @brief 获取红色LED
// *
// * @return Enum_BSP_LED_Status 状态
// */
//Enum_BSP_LED_Status BSP_Get_Blue_LED()
//{
//   
//}

/**
 * @brief 获取按键
 *
 * @return Enum_BSP_Key_Status 状态
 */
Enum_BSP_Key_Status BSP_Get_Key()
{
    static GPIO_PinState pre_key_status;
    GPIO_PinState key_status;
    Enum_BSP_Key_Status return_value = BSP_Key_Status_FREE;

    key_status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

    switch (pre_key_status)
    {
    case (GPIO_PIN_RESET):
        {
            switch (key_status)
            {
            case (GPIO_PIN_RESET):
                {
                    pre_key_status = key_status;
                    return_value = BSP_Key_Status_FREE;

                    break;
                }
            case (GPIO_PIN_SET):
                {
                    pre_key_status = key_status;
                    return_value = BSP_Key_Status_TRIG_FREE_TO_PRESSED;

                    break;
                }
            }

            break;
        }
    case (GPIO_PIN_SET):
        {
            switch (key_status)
            {
            case (GPIO_PIN_RESET):
                {
                    pre_key_status = key_status;
                    return_value = BSP_Key_Status_TRIG_PRESSED_TO_FREE;

                    break;
                }
            case (GPIO_PIN_SET):
                {
                    pre_key_status = key_status;
                    return_value = BSP_Key_Status_PRESSED;

                    break;
                }
            }

            break;
        }
    }

    return (return_value);
}


/**
 * @brief 设定蜂鸣器
 *
 * @param Rate 蜂鸣器响度占空比
 */
void BSP_Set_PWM_Buzzer(float Rate)
 {
     __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, Rate * 1250);
 }
