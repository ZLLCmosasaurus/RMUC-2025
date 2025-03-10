#include "dvc_ws2812.h"

uint8_t Color_RED[3]= {255,255,255};

void Class_WS2812::Init(TIM_HandleTypeDef *__Driver_PWM_TIM,uint8_t __Driver_PWM_TIM_Channel)
{
    Driver_PWM_TIM  =   __Driver_PWM_TIM;
    Driver_PWM_TIM_Channel  =   __Driver_PWM_TIM_Channel;
    HAL_TIM_PWM_Start_DMA(Driver_PWM_TIM,Driver_PWM_TIM_Channel,(uint32_t*)Send_Buffer,Length);
}

void Class_WS2812::Load(void)
{
    HAL_TIM_PWM_Start_DMA(Driver_PWM_TIM,Driver_PWM_TIM_Channel,(uint32_t*)Send_Buffer,Length);
}

void Class_WS2812::CloseAll(void)
{
    uint16_t i;
    for ( i = 0; i < LED_Num*24; i++)
    {
        Send_Buffer[i]  =   WS2812_0_Pulse;
    }
    for ( i = LED_Num*24; i < Buffer_Length; i++)
    {
        Send_Buffer[i]  =   0;
    }
    Load();
}

void Class_WS2812::WriteAll(uint8_t* __color)
{
    uint16_t i,j;
    uint8_t dat[24];

   for (i = 0; i < 8; i++)
	{
		dat[i] = ((__color[0] & 0x80) ? WS2812_1_Pulse : WS2812_0_Pulse);
		__color[0] <<= 1;
	}
	for (i = 0; i < 8; i++)
	{
		dat[i + 8] = ((__color[1] & 0x80) ? WS2812_1_Pulse : WS2812_0_Pulse);
		__color[1] <<= 1;
	}
	for (i = 0; i < 8; i++)
	{
		dat[i + 16] = ((__color[2] & 0x80) ? WS2812_1_Pulse : WS2812_0_Pulse);
		__color[2] <<= 1;
	}
	for (i = 0; i < LED_Num; i++)
	{
		for (j = 0; j < 24; j++)
		{
			Send_Buffer[i * 24 + j] = dat[j];
		}
	}
	for (i = LED_Num * 24; i < Buffer_Length; i++)
    Send_Buffer[i] = 0; // 占空比比为0，全为低电平
    Load();
}

