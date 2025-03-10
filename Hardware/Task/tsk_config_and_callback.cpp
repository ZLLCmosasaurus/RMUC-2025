
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
        case (0x66):
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

void Task1ms_TIM2_Callback()                    //正常状态发送数据
{                                   
    //处理数据类型
    int send_p_motor;                   
    uint8_t send_mode;              //控制机状态
    uint16_t send_buffer_p;         //发送缓冲功率
	uint16_t send_surplus_energy;    //发送百分比
    send_p_motor=measure.P_Motor*100;
    send_buffer_p=0;
    send_surplus_energy=measure.surplus_energy*100;
    send_mode=control.Cap_Mode;
    memcpy(&SEND_DATA[0],&send_p_motor,2);          //底盘功率
    memcpy(&SEND_DATA[2],&send_buffer_p,2);         //缓冲功率
    memcpy(&SEND_DATA[4],&send_surplus_energy,2);   //电容组电量百分比
    memcpy(&SEND_DATA[6],&send_mode,1);             //超电状态
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
        ADC_init();                        //初始化ADC
        HRTIM_Init();                      //初始化HRTIM定时器
        PID_init(&control.currout_loop, 0.0046f, 0.005f, 0.f, 0.0f,MAX_DUTY, 0.005f , MIN_DUTY, -0.005f);     //DC-DC电流环
        PID_init(&control.voltout_loop,0.0046f, 0.005f, 0.f, 0.0f,MAX_DUTY, 0.01f , MIN_DUTY, -0.01f);     //电容电压环
        //kf无需初始化，后续会计算得到
        // PID_init(&control.powerin_loop, 10.0f, 0.0f,0.0f, 120, 100, -300, -100);                 //DCDC功率环
        while(Power_on_Self_Test());            //模式正常才会结束
    //定时器循环任务
  
	TIM_Init(&htim2, Task1ms_TIM2_Callback);            //定时器中断can信号
    
    /********************************* 设备层初始化 *********************************/

    //设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/
 //   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);


    /********************************* 使能调度时钟 *********************************/

    HAL_TIM_Base_Start_IT(&htim2);                                      //开启CAN发送定时时钟
	__HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1,HRTIM_TIMERINDEX_TIMER_C,HRTIM_TIM_IT_REP);        //中段执行函数在 HRTIM1_TIMC_IRQHandler
}

/**
 * @brief 前台循环任务
 *
 */
 extern "C" void Task_Loop()
{   

    my_key();
    Bluetooth_transmission1();                  //蓝牙传输
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


 extern "C" void HRTIM1_TIMC_IRQHandler(void)       //主任务执行
{
  /* USER CODE BEGIN HRTIM1_TIMC_IRQn 0 */

  /* USER CODE END HRTIM1_TIMC_IRQn 0 */
  HAL_HRTIM_IRQHandler(&hhrtim1,HRTIM_TIMERINDEX_TIMER_C);
  /* USER CODE BEGIN HRTIM1_TIMC_IRQn 1 */
//	get_time_ms1=DWT_GetTimeline_ms();

	ADC_Measure();
    Mode_Judgment();
    PID_Control();
  

//  get_time_ms2=DWT_GetTimeline_ms();
 // get_time=get_time_ms2-get_time_ms1;
  /* USER CODE END HRTIM1_TIMC_IRQn 1 */
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
