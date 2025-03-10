#ifndef DVC_WS2812
#define DVC_WS2812

#include "drv_tim.h"

#define LED_Num 66
#define Reset_Buffer_Length 300         //Reset 280us/1.25us=224
#define Buffer_Length (LED_Num*24+Reset_Buffer_Length)
#define WS2812_1_Pulse  100
#define WS2812_0_Pulse  35

extern uint8_t Color_RED[3];

class Class_WS2812
{
    public:
    
    
    
    void Init(TIM_HandleTypeDef *__Driver_PWM_TIM,uint8_t __Driver_PWM_TIM_Channel);
    void Load(void);
    void CloseAll(void);
    void WriteAll(uint8_t* __color);

    protected:
    TIM_HandleTypeDef *Driver_PWM_TIM;
    uint8_t Driver_PWM_TIM_Channel;
    uint16_t Length =   Buffer_Length;
    uint16_t Send_Buffer[Buffer_Length];
};

#endif // !DVC_WS2812
