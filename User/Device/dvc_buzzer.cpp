#include "dvc_buzzer.h"

#define BUZZER_FORCE_STOP_PRIORITY_NUM 10
#define BUZZER_DEVICE_OFFLINE_PRIORITY_NUM 10
#define BUZZER_CALIBRATING_PRIORITY_NUM 5
#define BUZZER_CALIBRATED_PRIORITY_NUM 10
#define BUZZER_DJI_STARTUP_PRIORITY_NUM 3
#define BUZZER_FREE_PRIORITY_NUM 10

int BUZZER_FORCE_STOP_PRIORITY_TONELIST[2][BUZZER_FORCE_STOP_PRIORITY_NUM] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                                                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
int BUZZER_DEVICE_OFFLINE_PRIORITY_TONELIST[2][BUZZER_DEVICE_OFFLINE_PRIORITY_NUM] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                                                                         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
int BUZZER_CALIBRATING_PRIORITY_TONELIST[2][BUZZER_CALIBRATING_PRIORITY_NUM] = {{TONE_A3, TONE_A4, TONE_A3,TONE_D4, TONE_C5},
                                                                                   {1000,2000, 3000, 4000, 8000}};
int BUZZER_CALIBRATED_PRIORITY_TONELIST[2][BUZZER_CALIBRATED_PRIORITY_NUM] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                                                                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
int BUZZER_DJI_STARTUP_PRIORITY_TONELIST[2][BUZZER_DJI_STARTUP_PRIORITY_NUM] = {{TONE_A3, TONE_C4,TONE_C5},
                                                                                    {3000, 6000, 9000}};
int BUZZER_FREE_PRIORITY_TONELIST[2][BUZZER_FREE_PRIORITY_NUM] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                                                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

void Class_Buzzer::Buzzer_Init(TIM_HandleTypeDef *__Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel,uint32_t __Driver_Frequency) {
    Driver_PWM_TIM = __Driver_PWM_TIM;
    Driver_PWM_TIM_Channel = __Driver_PWM_TIM_Channel;
    ClkFreq = __Driver_Frequency;
    Psc =   Driver_PWM_TIM->Instance->PSC;
    Arr =   Driver_PWM_TIM->Instance->ARR;
    Buzzer_SheetListInit();
    NowTask= BUZZER_FREE_PRIORITY;
}

void Class_Buzzer::Buzzer_On(void) {
    HAL_TIM_PWM_Start(Driver_PWM_TIM, Driver_PWM_TIM_Channel);
}

void Class_Buzzer::Buzzer_Off(void) {
    HAL_TIM_PWM_Stop(Driver_PWM_TIM, Driver_PWM_TIM_Channel);
}

void Class_Buzzer::Set_State(BUZZER_SWITCH_STATE_E __State) {
    BuzzerState   =   __State;
    if (__State == BUZZER_SWITCH_ON) {
        Buzzer_On();
    } else {
        Buzzer_Off();
    }
}

void Class_Buzzer::Set_Arr(uint16_t __Arr) {
    Arr = __Arr;
    Driver_PWM_TIM->Instance->ARR = Arr;
    __HAL_TIM_SetCompare(Driver_PWM_TIM, Driver_PWM_TIM_Channel, Arr/3*2);
}

void Class_Buzzer::Set_NowTime(float __NowTime) {
    BuzzerNowTime = __NowTime;
}

void Class_Buzzer::Set_StartTime(float __StartTime) {
    BuzzerStartTime = __StartTime;
}

void Class_Buzzer::Set_TargetFrequency(float __TargetFrequency) {
    TargetFrequency = __TargetFrequency;
}

void Class_Buzzer::Play_Tone(BUZZER_BSP_TONE_TO_POW_LUT_H __Tone){
    Set_TargetFrequency(TONE_A4_FREQUENCY*pow(SEMITONE_COEFFICIENT, __Tone));
    Set_Arr((uint16_t)(ClkFreq / Psc / TargetFrequency));
}

void Class_Buzzer::Reload_Buzzer_Status_PeriodElapsedCallback(void) {
    if(Get_NowTask()!=Get_LastTask()&&Get_NowTask()!=BUZZER_FREE_PRIORITY) {
        Set_State(BUZZER_SWITCH_ON);
        Set_StartTime(HAL_GetTick());
    }
}

void Class_Buzzer::Buzzer_Calculate_PeriodElapsedCallback(void) {
  Reload_Buzzer_Status_PeriodElapsedCallback();
  switch(Get_NowTask()) {
    case BUZZER_FREE_PRIORITY:
        Set_State(BUZZER_SWITCH_OFF);
      break;
    case BUZZER_FORCE_STOP_PRIORITY:
        PlayToneList(BUZZER_FORCE_STOP_PRIORITY);
      break;
    case BUZZER_DEVICE_OFFLINE_PRIORITY:
        PlayToneList(BUZZER_DEVICE_OFFLINE_PRIORITY);
      break;
    case BUZZER_CALIBRATING_PRIORITY:
        PlayToneList(BUZZER_CALIBRATING_PRIORITY);
      break;
    case BUZZER_CALIBRATED_PRIORITY:
        PlayToneList(BUZZER_CALIBRATED_PRIORITY);
      break;
    case BUZZER_DJI_STARTUP_PRIORITY:
        PlayToneList(BUZZER_DJI_STARTUP_PRIORITY);
      break;
   
  }
  Set_LastTask(Get_NowTask());
}

void Class_Buzzer::Set_WaitTime(void) {
    BuzzerWaitTime = BuzzerNowTime - BuzzerStartTime;
  }
float Class_Buzzer::Get_WaitTime(void) {
    return BuzzerWaitTime;
  }
uint8_t i = 0;
void Class_Buzzer::PlayToneList(BUZZER_BEEP_TASK_PRIORITY_E __NowTask){
    
    Set_NowTime(HAL_GetTick());
    Set_WaitTime();
    if((Get_WaitTime()<SheetMusic[__NowTask].ToneList[i].GetDuration()+BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE)   && i    <   SheetMusic[__NowTask].GetNum()) {
        Play_Tone(SheetMusic[__NowTask].ToneList[i].GetTone());
        
    }
    else if(Get_WaitTime()>=SheetMusic[__NowTask].ToneList[i].GetDuration()+BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE  && i < SheetMusic[__NowTask].GetNum()){
        i++;
    }
    else{
        Set_State(BUZZER_SWITCH_OFF);
        NowTask= BUZZER_FREE_PRIORITY;
				i=0;
    }
    
}

void Class_Buzzer::Set_NowTask(BUZZER_BEEP_TASK_PRIORITY_E __NowTask) {
    
        if(__NowTask<=NowTask) {
        NowTask=__NowTask;
        }
  
  }
void Class_Buzzer::Set_LastTask(BUZZER_BEEP_TASK_PRIORITY_E __LastTask){
    LastTask = __LastTask;
  }

   BUZZER_BEEP_TASK_PRIORITY_E Class_Buzzer::Get_NowTask(void){
    return NowTask;
  }
  BUZZER_BEEP_TASK_PRIORITY_E Class_Buzzer::Get_LastTask(void){
    return LastTask;
  }

void Class_Buzzer_Tone::SetTone(BUZZER_BSP_TONE_TO_POW_LUT_H __Tone){
      Tone  =   __Tone;
  }

void Class_Buzzer_Tone::SetDuration(float __Duration){
      Duration  =   __Duration;
  }

  BUZZER_BSP_TONE_TO_POW_LUT_H Class_Buzzer_Tone::GetTone(void){
      return Tone;
  }

float Class_Buzzer_Tone::GetDuration(void){
      return Duration;
  }

void Class_Sheet_Music::SetNum(uint8_t __Num){
    Num = __Num;
}

uint8_t Class_Sheet_Music::GetNum(void){
    return Num;
}

void Class_Sheet_Music::SetToneList(int *__Tone, int *__Duration, uint8_t __Num){
    for (size_t i = 0; i < __Num; i++)
    {
        ToneList[i].SetTone((BUZZER_BSP_TONE_TO_POW_LUT_H)__Tone[i]);
        ToneList[i].SetDuration(__Duration[i]);
    }
    SetNum(__Num);
  }

void Class_Buzzer::Buzzer_SheetListInit(void){
    memset(SheetMusic, 0, sizeof(SheetMusic));
    SheetMusic[BUZZER_FORCE_STOP_PRIORITY].SetToneList(BUZZER_FORCE_STOP_PRIORITY_TONELIST[0], BUZZER_FORCE_STOP_PRIORITY_TONELIST[1], BUZZER_FORCE_STOP_PRIORITY_NUM);
    SheetMusic[BUZZER_DEVICE_OFFLINE_PRIORITY].SetToneList(BUZZER_DEVICE_OFFLINE_PRIORITY_TONELIST[0], BUZZER_DEVICE_OFFLINE_PRIORITY_TONELIST[1], BUZZER_DEVICE_OFFLINE_PRIORITY_NUM);
    SheetMusic[BUZZER_CALIBRATING_PRIORITY].SetToneList(BUZZER_CALIBRATING_PRIORITY_TONELIST[0], BUZZER_CALIBRATING_PRIORITY_TONELIST[1], BUZZER_CALIBRATING_PRIORITY_NUM);
    SheetMusic[BUZZER_CALIBRATED_PRIORITY].SetToneList(BUZZER_CALIBRATED_PRIORITY_TONELIST[0], BUZZER_CALIBRATED_PRIORITY_TONELIST[1], BUZZER_CALIBRATED_PRIORITY_NUM);
    SheetMusic[BUZZER_DJI_STARTUP_PRIORITY].SetToneList(&BUZZER_DJI_STARTUP_PRIORITY_TONELIST[0][0], BUZZER_DJI_STARTUP_PRIORITY_TONELIST[1], BUZZER_DJI_STARTUP_PRIORITY_NUM);
    SheetMusic[BUZZER_FREE_PRIORITY].SetToneList(BUZZER_FREE_PRIORITY_TONELIST[0], BUZZER_FREE_PRIORITY_TONELIST[1], BUZZER_FREE_PRIORITY_NUM);
}