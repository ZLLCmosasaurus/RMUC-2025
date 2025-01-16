#include "stm32f4xx_hal.h"
#include "dvc_dwt.h"
#include "math.h"
#include "cstring"

// 任务优先级数量上限
#define BUZZER_MAX_PRIORITY_NUM 16
#define BUZZER_MAX_TONE_NUM 32
// 允许音调使用时间点和进入时间点的最大差值
#define BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE 20
// 用于规定各种蜂鸣器发声任务优先级，优先进行高优先级的发声任务
// （数字越小优先级越高）





#define Limit_Buzzer_Frequency(target,measure,limition) ((target-measure)<limition?0:1)

#define TONE_A4_FREQUENCY		440 // 标准音频率
#define SEMITONE_COEFFICIENT	1.0594630943592953 // 半音频率比例
#define LOWEST_TONE_POW			-11
#define HIGHEST_TONE_POW		12

// 用于查表确定次方数，从而计算蜂鸣器频率
typedef enum
{
	TONE_A3			= -12,
	TONE_A_SLASH_3	= -11,
	TONE_B3			= -10,
	TONE_C4			= -9,
	TONE_C_SLASH_4	= -8,
	TONE_D4			= -7,
	TONE_D_SLASH_4	= -6,
	TONE_E4			= -5,
	TONE_F4			= -4,
	TONE_F_SLASH_4	= -3,
	TONE_G4			= -2,
	TONE_G_SLASH_4	= -1,
	TONE_A4			= 0,
	TONE_A_SLASH_4	= 1,
	TONE_B4			= 2,
	TONE_C5			= 3,
	TONE_C_SLASH_5	= 4,
	TONE_D5			= 5,
	TONE_D_SLASH_5	= 6,
	TONE_E5			= 7,
	TONE_F5			= 8,
	TONE_F_SLASH_5	= 9,
	TONE_G5			= 10,
	TONE_G_SLASH_5	= 11,
	TONE_A5			= 12,
} BUZZER_BSP_TONE_TO_POW_LUT_H;

typedef uint8_t BUZZER_BEEP_TASK_PRIORITY_E;
enum {
  BUZZER_FORCE_STOP_PRIORITY = 0,
  BUZZER_DJI_STARTUP_PRIORITY = 5,
  BUZZER_CALIBRATING_PRIORITY = 2,
  BUZZER_CALIBRATED_PRIORITY = 3,
  BUZZER_DEVICE_OFFLINE_PRIORITY = 1,
  BUZZER_FREE_PRIORITY = BUZZER_MAX_PRIORITY_NUM, // 空闲状态
};

typedef enum
{
	BUZZER_SWITCH_ON	= 0,
	BUZZER_SWITCH_OFF	= 1,
} BUZZER_SWITCH_STATE_E;

class Class_Buzzer_Tone
{
  public:
    BUZZER_BSP_TONE_TO_POW_LUT_H Tone;
    float Duration;

    void SetTone(BUZZER_BSP_TONE_TO_POW_LUT_H __Tone);
    void SetDuration(float __Duration);
    BUZZER_BSP_TONE_TO_POW_LUT_H GetTone(void);
    float GetDuration(void);
};

class Class_Sheet_Music
{
  public:
  Class_Buzzer_Tone ToneList[BUZZER_MAX_TONE_NUM];
  BUZZER_BSP_TONE_TO_POW_LUT_H *Tone; 
  float *Duration;
  uint8_t Num;
  void SetToneList(int *__Tone, int *__Duration, uint8_t __Num);
  void PlayToneList(void);
  void SetNum(uint8_t __Num);
  uint8_t GetNum(void);
    
};

class Class_Buzzer
{
  public:
 
    void Buzzer_Init(TIM_HandleTypeDef *__Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel,uint32_t __Driver_Frequency=84000000);
    void Buzzer_SheetListInit(void);
    void Buzzer_On(void);
    void Buzzer_Off(void);
    
    TIM_HandleTypeDef *Driver_PWM_TIM;
    uint8_t Driver_PWM_TIM_Channel;

    uint32_t ClkFreq;
    uint16_t Psc;
    uint16_t Arr;

    BUZZER_SWITCH_STATE_E BuzzerState;
    float BuzzerStartTime;
    float BuzzerNowTime;
    float BuzzerWaitTime;

    Class_Sheet_Music SheetMusic[BUZZER_MAX_PRIORITY_NUM];

    float TargetFrequency;

    BUZZER_BEEP_TASK_PRIORITY_E NowTask;
    BUZZER_BEEP_TASK_PRIORITY_E LastTask;

    void Play_Tone(BUZZER_BSP_TONE_TO_POW_LUT_H __Tone );
    void Set_Arr(uint16_t __Arr);
    void Set_NowTime(float __NowTime);
    void Set_StartTime(float __StartTime);
    void Set_WaitTime(void);
    float Get_WaitTime(void);
    void Set_State(BUZZER_SWITCH_STATE_E __State);
    void Set_TargetFrequency(float __TargetFrequency);
    void Set_NowTask(BUZZER_BEEP_TASK_PRIORITY_E __NowTask);
    void Set_LastTask(BUZZER_BEEP_TASK_PRIORITY_E __LastTask);
    BUZZER_BEEP_TASK_PRIORITY_E Get_NowTask(void);
    BUZZER_BEEP_TASK_PRIORITY_E Get_LastTask(void);
    void Buzzer_Calculate_PeriodElapsedCallback(void);
    void Reload_Buzzer_Status_PeriodElapsedCallback(void);
    void PlayToneList(BUZZER_BEEP_TASK_PRIORITY_E __Task);
};



