/**
 * @file tsk_config_and_callback.cpp
 * @author lez by yssickjgd
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 * @copyright ZLLC 2024
 */

/**
 * @brief 注意, 每个类的对象分为专属对象Specialized, 同类可复用对象Reusable以及通用对象Generic
 *
 * 专属对象:
 * 单对单来独打独
 * 比如交互类的底盘对象, 只需要交互对象调用且全局只有一个, 这样看来, 底盘就是交互类的专属对象
 * 这种对象直接封装在上层类里面, 初始化在上层类里面, 调用在上层类里面
 *
 * 同类可复用对象:
 * 各调各的
 * 比如电机的对象, 底盘可以调用, 云台可以调用, 而两者调用的是不同的对象, 这种就是同类可复用对象
 * 电机的pid对象也算同类可复用对象, 它们都在底盘类里初始化
 * 这种对象直接封装在上层类里面, 初始化在最近的一个上层专属对象的类里面, 调用在上层类里面
 *
 * 通用对象:
 * 多个调用同一个
 * 比如裁判系统对象, 底盘类要调用它做功率控制, 发射机构要调用它做出膛速度与射击频率的控制, 因此裁判系统是通用对象.
 * 这种对象以指针形式进行指定, 初始化在包含所有调用它的上层的类里面, 调用在上层类里面
 *
 */

/**
 * @brief TIM开头的默认任务均1ms, 特殊任务需额外标记时间
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "ita_chariot.h"
#include "drv_dwt.h"
#include "bsp_uart.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint32_t init_finished =0 ;
bool start_flag=0;
//机器人控制对象
Class_Chariot chariot;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x201):   
    {
        
    }
    break;
    case (0x202):   
    {
        
    }
    break;
    case (0x203):
    {
        chariot.Motor_left.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
       
    }
    break;
    case (0x206):
    {
        
    }
    break;
	}
}

/**
 * @brief CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
void Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x88):   
    {
        
    }
    break;
	}
}

/**
 * @brief SPI1回调函数
 *
 * @param Tx_Buffer SPI1发送的消息
 * @param Rx_Buffer SPI1接收的消息
 * @param Length 长度
 */
void Device_SPI1_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
{

}

/**
 * @brief UART3遥控器回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
uint16_t length;
void DR16_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    // if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) && 
	// 		__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE))
	// {
    //     /* clear idle it flag avoid idle interrupt all the time */
    //     __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    //     /* clear DMA transfer complete flag */
    //     __HAL_DMA_DISABLE(huart1.hdmarx);
    //     length = (uint16_t)(huart1.hdmarx->Instance->NDTR);
    //     /* handle dbus data dbus_buf from DMA */
    //     if ((50 - (uint16_t)(huart1.hdmarx->Instance->NDTR)) == UART1_Manage_Object.Rx_Length)
    //     {
            chariot.DR16.DR16_UART_RxCpltCallback(Buffer);

            //底盘 云台 发射机构 的控制策略
            chariot.TIM_Control_Callback();
    //     }
        
    //     /* restart dma transmission */
    //     __HAL_DMA_SET_COUNTER(huart1.hdmarx, DBUS_MAX_LEN);
    //     __HAL_DMA_ENABLE(huart1.hdmarx);
	// }

}

/**
 * @brief IIC磁力计回调函数
 *
 * @param Buffer IIC收到的消息
 * @param Length 长度
 */
void Ist8310_IIC3_Callback(uint8_t* Tx_Buffer, uint8_t* Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
    
}

/**
 * @brief UART裁判系统回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */
void Referee_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{
    //chariot.Referee.UART_RxCpltCallback(Buffer,Length);
}


/**
 * @brief USB MiniPC回调函数
 *
 * @param Buffer USB收到的消息
 *
 * @param Length 长度
 */

void MiniPC_USB_Callback(uint8_t *Buffer, uint32_t Length)
{
    //chariot.MiniPC.USB_RxCpltCallback(Buffer);
}

/**
 * @brief TIM5任务回调函数
 *
 */
extern "C" void Control_Task_Callback()
{
    init_finished++;
    if(init_finished>400)
    start_flag=1;

    /************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/
    
    chariot.TIM5msMod10_Alive_PeriodElapsedCallback();

    /****************************** 交互层回调函数 1ms *****************************************/
    if(start_flag==1)
    {

        chariot.FSM_Alive_Control.Reload_TIM_Status_PeriodElapsedCallback();
        chariot.TIM_Calculate_PeriodElapsedCallback();
        
    /****************************** 驱动层回调函数 1ms *****************************************/ 
        //统一打包发送
        TIM_CAN_PeriodElapsedCallback();

        TIM_UART_PeriodElapsedCallback();
        
        // static int mod5 = 0;
        // mod5++;
        // if (mod5 == 5)
        // {
        //     TIM_USB_PeriodElapsedCallback(&MiniPC_USB_Manage_Object);
        // mod5 = 0;
        // }	        
    }
}

/**
 * @brief 初始化任务
 *
 */
extern "C" void Task_Init()
{  

    DWT_Init(168);

    /********************************** 驱动层初始化 **********************************/

    //裁判系统
    UART_Init(&huart6, Referee_UART6_Callback, 128);   //并未使用环形队列 尽量给长范围增加检索时间 减少丢包

    //遥控器接收
    UART_Init(&huart1, DR16_UART1_Callback, 18);

    //集中总线can1/can2
    CAN_Init(&hcan1, Device_CAN1_Callback);
    CAN_Init(&hcan2, Device_CAN2_Callback);

    // //c板陀螺仪spi外设
    // SPI_Init(&hspi1,Device_SPI1_Callback);

    // //磁力计iic外设
    // IIC_Init(&hi2c3, Ist8310_IIC3_Callback);    

    // UART_Init(&huart6, Image_UART6_Callback, 40);

    //上位机USB
    // USB_Init(&MiniPC_USB_Manage_Object,MiniPC_USB_Callback);

    // HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

    //定时器循环任务
    // TIM_Init(&htim4, Task100us_TIM4_Callback);
    // TIM_Init(&htim5, Task1ms_TIM5_Callback);

    /********************************* 设备层初始化 *********************************/

    //设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/

    chariot.Init();

    /********************************* 使能调度时钟 *********************************/

    // HAL_TIM_Base_Start_IT(&htim4);
    // HAL_TIM_Base_Start_IT(&htim5);
}

/**
 * @brief 前台循环任务
 *
 */
 extern "C" void Task_Loop()
{

}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
