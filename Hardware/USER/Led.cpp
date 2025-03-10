#include "stm32f3xx_hal.h"
#include "Led.h"
//PB12--G
//PB13--B
//PA14--G
//HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
//HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Led_Control(void)
{
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_15);
}
