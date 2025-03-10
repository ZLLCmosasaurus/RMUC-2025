
/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "drv_tim.h"
#include "config.h"
#include "drv_can.h"
#include "dvc_dwt.h"
#include "drv_uart.h"
#include "drv_tim.h"
#include "Key.h"
#include "Measure.h"
#include "Led.h"
#include "Communication.h"
#include "PID.h"
#include "Buzzer.h"
#include "Power_control.h"
#include "TIMER.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


void CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
        case (0x67):
        {
            CAN_Data_Process(CAN_RxMessage->Data);
        }
        break;
    }
}


/**
 * @brief TIM1任务回调函数
 *
 */

void Task1ms_TIM2_Callback()
{   
    int Send_P_Motor;   //处理后的数据
    int Send_V_Cap;
    Send_P_Motor=measure.P_Motor*100;
    Send_V_Cap=ADC_V_CAP.Solved_value*100;
    memcpy(&SEND_DATA[0],&Send_P_Motor,2);  //底盘功率
    memcpy(&SEND_DATA[3],&Send_V_Cap,2);  //电容组电量百分比
    CAN_Send_Data(&hcan,CAN_ID,SEND_DATA,8);
}

/**
 * @brief 初始化任务
 *
 */
extern "C" void Task_Init()
{  

    DWT_Init(72);

    /********************************** 驱动层初始化 **********************************/
        //初始化CAN
        CAN_Init(&hcan, CAN1_Callback);
        ADC_init();                 //初始化ADC
        HRTIM_Init();               //初始化HRTIM定时器
     //   PID_init(&control.currout_loop, 0.0046f, 0.0091f, 0, MAX_DUTY, 0.001f , MIN_DUTY, -0.001f);     //DC-DC电流环
     //   PID_init(&control.voltout_loop,0.0046f, 0.0091f, 0, MAX_DUTY, 0.001f , MIN_DUTY, -0.001f);     //电容电压环
     //   PID_init(&control.powerin_loop, 10.0f, 0.0f,0.0f, 120, 100, -300, -100);                 //DCDC功率环
            PID_init(&control.currout_loop, 0.0046f, 0.005f, 0.f, 0.0f,MAX_DUTY, 0.005f , MIN_DUTY, -0.005f);     //DC-DC电流环
            PID_init(&control.voltout_loop,0.0046f, 0.005f, 0.f, 0.0f,MAX_DUTY, 0.01f , MIN_DUTY, -0.01f);     //电容电压环
       // PID_init(&control.powerin_loop, 10.0f, 0.0f,0.0f, 120, 100, -300, -100);                 //DCDC功率环
        while(Power_on_Self_Test());
    //定时器循环任务
  
	TIM_Init(&htim2, Task1ms_TIM2_Callback);
    
    /********************************* 设备层初始化 *********************************/

    //设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/
 //   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);


    /********************************* 使能调度时钟 *********************************/

    HAL_TIM_Base_Start_IT(&htim2);                                      //开启CAN发送定时时钟
	__HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1,HRTIM_TIMERINDEX_TIMER_C,HRTIM_TIM_IT_REP);
}

/**
 * @brief 前台循环任务
 *
 */
 extern "C" void Task_Loop()
{   
   // Update_PWM(control.test_ratio);
    my_key();
    Bluetooth_transmission1();
    Bluetooth_transmission2();
    if(LED_Cnt>=1000)
    {
        Led_Control();
        LED_Cnt=0;
    }
    if(rx_cnt>=100)
    {
        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
    }
}


/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
