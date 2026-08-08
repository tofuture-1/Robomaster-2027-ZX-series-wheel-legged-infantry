#include "dc_source.h"


/*打开24V输出电源*/
void __24V_DC_Power_On(void)
{
  HAL_GPIO_WritePin(GPIOC , GPIO_PIN_13,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC , GPIO_PIN_14, GPIO_PIN_SET);
}

/*关闭5V输出电源*/
void __24V_DC_Power_Off(void)
{
  HAL_GPIO_WritePin(GPIOC , GPIO_PIN_13,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC , GPIO_PIN_14, GPIO_PIN_RESET);
}

void __5V_DC_Power_On(void)
{
	HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_15 , GPIO_PIN_SET);
}

void __5V_DC_Power_Off(void)
{
	HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_15, GPIO_PIN_RESET);
}
