#include "stm32f4xx_hal.h"

typedef enum
{
	BUZZER_SWITCH_ON	= 0,
	BUZZER_SWITCH_OFF	= 1,
} BUZZER_SWITCH_STATE_E;

class Class_Buzzer
{
  public:
    Class_Buzzer();
    void Buzzer_Init(void);
    void Buzzer_On(void);
    void Buzzer_Off(void);

  private:
    TIM_HandleTypeDef *Driver_PWM_TIM;
    uint8_t Driver_PWM_TIM_Channel;

    uint16_t Driver_PWM_TIM_Pulse;
    uint16_t Driver_PWM_TIM_Prescale;
    uint16_t Driver_PWM_TIM_Arr;
    uint32_t Driver_PWM_TIM_Clock;

    uint32_t Driver_Tick;

    BUZZER_SWITCH_STATE_E Buzzer_State;

};



