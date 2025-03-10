#include "stm32f3xx_hal.h"
#include "Buzzer.h"

void BUZZER(uint32_t frequency)
{
    if (frequency == 0)
    {
        // 关闭蜂鸣器
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        return;
    }

    // 计算定时器的自动重装载值（ARR）和捕获/比较值（CCR）
    uint32_t period = (uint32_t)(HAL_RCC_GetPCLK1Freq() / frequency) - 1;
    uint32_t pulse = period / 2;  // 占空比 50%

    // 设置定时器参数
    __HAL_TIM_SET_AUTORELOAD(&htim3, period);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse);
}