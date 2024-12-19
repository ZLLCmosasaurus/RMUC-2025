/**
 * @file ita_chariot.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_chariot.h"
#include "drv_math.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init(float __DR16_Dead_Zone)
{ 
    //裁判系统
    //Referee.Init(&huart6);

    //yaw电机canid初始化  只获取其编码器值用于底盘随动，并不参与控制
    Motor_Yaw.Init(&hcan2, DJI_Motor_ID_0x205);

    Motor_down.Init(&hcan1, DJI_Motor_ID_0x201);
    Motor_up.Init(&hcan1, DJI_Motor_ID_0x202);
    Motor_left.Init(&hcan1, DJI_Motor_ID_0x203);
    Motor_right.Init(&hcan1, DJI_Motor_ID_0x204);

    //TensionMeter初始化
    Tension_Meter.Init(GPIOF, GPIO_PIN_1, GPIOF, GPIO_PIN_0);

    Servo_Load_1.Init(&htim5,TIM_CHANNEL_1);
    Servo_Load_2.Init(&htim5,TIM_CHANNEL_2);
    Servo_Load_3.Init(&htim5,TIM_CHANNEL_3);
    Servo_Load_4.Init(&htim5,TIM_CHANNEL_4);

    Servo_Trigger.Init(&htim4,TIM_CHANNEL_4);

    //遥控器离线控制 状态机
    FSM_Alive_Control.Chariot = this;
    FSM_Alive_Control.Init(5, 0);

    //遥控器
    DR16.Init(&huart1,&huart6);
    DR16_Dead_Zone = __DR16_Dead_Zone;   
            
    //上位机
    // MiniPC.Init(&MiniPC_USB_Manage_Object);
    // MiniPC.Referee = &Referee;
}

/**
 * @brief 计算回调函数
 *
 */

void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{

}
/**
 * @brief 控制回调函数
 *
 */
void Class_Chariot::TIM_Control_Callback()
{

}
/**
 * @brief 在线判断回调函数
 *
 */
void Class_Chariot::TIM5msMod10_Alive_PeriodElapsedCallback()
{
    static uint8_t mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
        Motor_Yaw.TIM_Alive_PeriodElapsedCallback();  
        Motor_up.TIM_Alive_PeriodElapsedCallback();       
        Motor_down.TIM_Alive_PeriodElapsedCallback();
        Motor_left.TIM_Alive_PeriodElapsedCallback();
        Motor_right.TIM_Alive_PeriodElapsedCallback();
        Tension_Meter.TIM_Alive_PeriodElapsedCallback();
        //MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();
        //Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
        mod10 = 0;
    }    
}


void Class_FSM_Dart_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;
    switch (Now_Status_Serial)
    {
        case Dart_Init_Status:
        {

        }
        break;
        case 1:
        {

        }
        break;
        case 2:
        {

        }
        default:
            break;
    }
}

/**
 * @brief 机器人遥控器离线控制状态转移函数
 *
 */
void Class_FSM_Alive_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
        // 离线检测状态
        case (0):
        {
            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart1.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }

            //转移为 在线状态
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {             
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            //超过一秒的遥控器离线 跳转到 遥控器关闭状态
            if(Status[Now_Status_Serial].Time > 1000)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(1);
            }
        }
        break;
        // 遥控器关闭状态
        case (1):
        {
            //离线保护

            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {
                Set_Status(2);
            }

            //遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart1.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }
            
        }
        break;
        // 遥控器在线状态
        case (2):
        {
            //转移为 刚离线状态
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(3);
            }
        }
        break;
        //刚离线状态
        case (3):
        {
            //记录离线检测前控制模式

            //无条件转移到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
        //遥控器串口错误状态
        case (4):
        {
            HAL_UART_DMAStop(&huart1); // 停止以重启
            //HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);

            //处理完直接跳转到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
    } 
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
