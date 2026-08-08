#ifndef __DC_SOURCE_H
#define __DC_SOURCE_H

#include "cstdint"
#include "stm32h7xx_hal.h"


void __5V_DC_Power_On();
void __5V_DC_Power_Off();
void __24V_DC_Power_On();
void __24V_DC_Power_Off();


#endif // DC_SOURCE_H
