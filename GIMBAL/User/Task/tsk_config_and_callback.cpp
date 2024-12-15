/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
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
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "buzzer.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint32_t init_finished = 0;
bool start_flag = 0;
// 机器人控制对象
Class_Chariot chariot;

// 串口裁判系统对象
Class_Serialplot serialplot;

// 上位机USB通讯对象
Struct_USB_Manage_Object MiniPC_USB_Manage_Object = {0};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Chassis_CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
#ifdef CHASSIS
void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x201):
    {
        chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
        chariot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        chariot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
        chariot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x206):
    {
    }
    break;
    case (0x207):
    {
    }
    break;
    }
}
#endif
/**
 * @brief Chassis_CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
#ifdef CHASSIS
void Chassis_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x206): // 留给yaw电机编码器回传 用于底盘随动
    {
        chariot.Motor_Yaw.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x77): // 留给上板通讯
    {
        chariot.CAN_Chassis_Control_RxCpltCallback();
    }
    break;
    case (0x204): // 留给超级电容
    {
    }
    break;
    case (0x205):
    {
        
    }
    break;
    }
}
#endif
/**
 * @brief Gimbal_CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
#ifdef GIMBAL
void Gimbal_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {

    case (0x201):
    {
        chariot.Booster.Motor_Friction_Left.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
        chariot.Booster.Motor_Friction_Right.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;

        break;
    case (0x141):
    {
        chariot.Gimbal.Motor_Pitch_LK6010.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}
#endif
/**
 * @brief Gimbal_CAN1回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
#ifdef GIMBAL
void Gimbal_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    
    switch (CAN_RxMessage->Header.StdId)
    {

    case (0x1fe): // 留给下板通讯
    {
        
        chariot.Referee.CAN_RxCpltCallback(CAN_RxMessage->Data,CAN_RxMessage->Header.StdId);
        
    }
    break;
    case (0x200): // 留给下板通讯
    {
        chariot.Referee.CAN_RxCpltCallback(CAN_RxMessage->Data,CAN_RxMessage->Header.StdId);
    }
    break;
    case (0x203):
    {
        chariot.Booster.Motor_Driver.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;

    case (0x204):
    {
        
    }
    break;
    case (0x205):
    {
        chariot.Gimbal.Motor_Pitch.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x206):
    {
        chariot.Gimbal.Motor_Yaw.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}
#endif
/**
 * @brief SPI5回调函数
 *
 * @param Tx_Buffer SPI5发送的消息
 * @param Rx_Buffer SPI5接收的消息
 * @param Length 长度
 */
// void Device_SPI5_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
//{
//     if (SPI5_Manage_Object.Now_GPIOx == BoardA_MPU6500_CS_GPIO_Port && SPI5_Manage_Object.Now_GPIO_Pin == BoardA_MPU6500_CS_Pin)
//     {
//         boarda_mpu.SPI_TxRxCpltCallback(Tx_Buffer, Rx_Buffer);
//     }
// }

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
 * @brief UART1图传链路回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
#ifdef GIMBAL
void Transmission_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.Referee.UART_RxCpltCallback(Buffer);

}
#endif

/**
 * @brief UART3遥控器回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
#ifdef GIMBAL
void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.DR16.UART_RxCpltCallback(Buffer);
	
	    // 底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();

}
#endif

/**
 * @brief IIC磁力计回调函数
 *
 * @param Buffer IIC收到的消息
 * @param Length 长度
 */
void Ist8310_IIC3_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
}

/**
 * @brief UART裁判系统回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */
#ifdef CHASSIS
void Referee_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.Referee.UART_RxCpltCallback(Buffer);
}
#endif
/**
 * @brief UART1超电回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
#if defined CHASSIS && defined POWER_LIMIT
void SuperCAP_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.Supercap.UART_RxCpltCallback(Buffer);
}
#endif
/**
 * @brief USB MiniPC回调函数
 *
 * @param Buffer USB收到的消息
 *
 * @param Length 长度
 */
#ifdef GIMBAL
void MiniPC_USB_Callback(uint8_t *Buffer, uint32_t Length)
{
    chariot.MiniPC.USB_RxCpltCallback(Buffer);
}
#endif
/**
 * @brief TIM4任务回调函数
 *
 */
void Task100us_TIM3_Callback()
{
#ifdef CHASSIS

    // 暂无云台tim4任务

#elif defined(GIMBAL)
    // 单给IMU消息开的定时器 ims
    chariot.Gimbal.Boardc_BMI.TIM_Calculate_PeriodElapsedCallback();

#endif
}



/**
 * @brief TIM5任务回调函数
 *
 */

uint16_t pwmVal = 300;
void Task1ms_TIM5_Callback()
{
    init_finished++;
    if(init_finished <3000)
		{
			buzzer_setTask(&buzzer,BUZZER_CALIBRATING_PRIORITY);
		}
    if (init_finished > 3000)//等待IMU稳定后开始控制
        start_flag = 1;
		

    /************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/

    chariot.TIM1msMod50_Alive_PeriodElapsedCallback();
  
    /****************************** 交互层回调函数 1ms *****************************************/
    if (start_flag == 1)
    {
        chariot.TIM_Calculate_PeriodElapsedCallback();
			  //buzzer_setTask(&buzzer,BUZZER_CALIBRATED_PRIORITY);

        /****************************** 驱动层回调函数 1ms *****************************************/

        // 统一打包发送
        TIM_CAN_PeriodElapsedCallback();

        TIM_UART_PeriodElapsedCallback();
		
	   
#ifdef GIMBAL
        //        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwmVal);
#endif

        static int mod5 = 0;
        mod5++;
        if (mod5 == 5)
        {
            TIM_USB_PeriodElapsedCallback(&MiniPC_USB_Manage_Object);
            mod5 = 0;
        }
    }
}
void Task1ms_TIM6_Callback(){
   // buzzer_taskScheduler(&buzzer);
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{

    DWT_Init(168);

/********************************** 驱动层初始化 **********************************/
#ifdef CHASSIS

    // 集中总线can1/can2
    CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
    CAN_Init(&hcan2, Chassis_Device_CAN2_Callback);

    // 裁判系统
    UART_Init(&huart6, Referee_UART6_Callback, 128);

#ifdef POWER_LIMIT
    // 旧版超电
    UART_Init(&huart1, SuperCAP_UART1_Callback, 128);
#endif

#endif

#ifdef GIMBAL

    // 集中总线can1/can2
    CAN_Init(&hcan1, Gimbal_Device_CAN1_Callback);
    CAN_Init(&hcan2, Gimbal_Device_CAN2_Callback);

    // c板陀螺仪spi外设
    SPI_Init(&hspi1, Device_SPI1_Callback);

    // 磁力计iic外设
    IIC_Init(&hi2c3, Ist8310_IIC3_Callback);

    // 遥控器接收
    UART_Init(&huart3, DR16_UART3_Callback, 18);

    // 上位机USB
    USB_Init(&MiniPC_USB_Manage_Object, MiniPC_USB_Callback);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    buzzer_init_example();

#endif

    // 定时器循环任务
    TIM_Init(&htim3, Task100us_TIM3_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);
     TIM_Init(&htim6, Task1ms_TIM6_Callback);

    /********************************* 设备层初始化 *********************************/

    // 设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/

    chariot.Init();

    /********************************* 使能调度时钟 *********************************/

    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);

    init_finished = 1;
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{
}


/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
