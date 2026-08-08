//
// Created by 谭恩泽 on 2025/10/24.
//

#ifndef C_BOARD_DRV_ADC_H
#define C_BOARD_DRV_ADC_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
/* Exported macros -----------------------------------------------------------*/

// ADC存储字节数, 与扫描内容有关
//#define SAMPLE_BUFFER_SIZE 4

///* Exported types ------------------------------------------------------------*/

///**
// * @brief ADC采样信息结构体
// *
// */
//struct Struct_ADC_Manage_Object
//{
//    ADC_HandleTypeDef *ADC_Handler;
//    uint16_t ADC_Data[SAMPLE_BUFFER_SIZE];
//};

///* Exported variables --------------------------------------------------------*/

//extern Struct_ADC_Manage_Object ADC1_Manage_Object;
//extern Struct_ADC_Manage_Object ADC2_Manage_Object;
//extern Struct_ADC_Manage_Object ADC3_Manage_Object;

///* Exported function declarations --------------------------------------------*/

//void ADC_Init(ADC_HandleTypeDef *hadc, uint16_t Sample_Number);

#endif //C_BOARD_DRV_ADC_H