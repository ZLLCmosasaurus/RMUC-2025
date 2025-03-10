#include "stm32f3xx_hal.h"
#include "TIMER.h"
#include "tim.h"
#include "hrtim.h"
#include "Communication.h"

void HRTIM_Init(void)
{
	HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER|HRTIM_TIMERID_TIMER_C| HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_A);
	//开启各定时器     开关频率350K
}



