#include "stm32f3xx_hal.h"
#include "Key.h"
#include "hrtim.h"
GPIO_PinState KEY1;
GPIO_PinState KEY2;
extern control_struct_t control; 
int PWM_CONTROL;
/*------------------------------------------------------------------------------------------------------
【函    数】按键调试函数1
【功    能】                
【参    数】
【返 回 值】
【注意事项】                         
-------------------------------------------------------------------------------------------------------*/

void my_key()
{	
	KEY1=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
	KEY2=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7);
	static uint8_t Key_Flag_1=1;
	static uint8_t Key_Flag_2=1;    

	
	if(Key_Flag_1&&(KEY1==0))
	{
		HAL_Delay(10);
		Key_Flag_1=0;
    if(KEY1==0)      
    {
		control.P_set+=20;	
		ratio_test+=0.01;
    }
	} 
	if(KEY1==1)
	{
		Key_Flag_1=1; 	
	} 

	if(Key_Flag_2&&(KEY2==0))
	{
		HAL_Delay(10);
		Key_Flag_2=0;	
		if(KEY2 == 0)
		{
			control.P_set-=20;
			ratio_test+=0.01;
			if(control.P_set<=0)
			{
				control.P_set=0;
			}
			if(ratio_test<=0)
			{
			ratio_test=0;
			}
		}
	}			
	if(KEY2==1)
	{
		Key_Flag_2=1; 
	}
}
