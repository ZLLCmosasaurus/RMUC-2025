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
#include "drv_bsp-boarda.h"
#include "drv_tim.h"
//#include "dvc_boarda-mpuahrs.h"
#include "dvc_boardc_bmi088.h"
#include "dvc_dmmotor.h"
#include "dvc_serialplot.h"
#include "ita_chariot.h"
#include "dvc_boardc_ist8310.h"
#include "dvc_imu.h"
#include "drv_usb.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "config.h"
#include "dvc_GraphicsSendTask.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint32_t init_finished =0 ;
bool start_flag=0;
//机器人控制对象
Class_Chariot chariot;

//串口裁判系统对象
Class_Serialplot serialplot;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */

void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
   if(CAN_RxMessage->Header.IDE== CAN_ID_STD)
    {
        switch (CAN_RxMessage->Header.StdId)
        {
        #ifdef CHASSIS
		    case(0x205):
		    {
		    	chariot.Chassis.Yaw_Motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
		    }
            break;
            case (0x23):
            {				
                // 500HZ			            
               chariot.Chassis.Joint_Motor[2].motor.CAN_RxCpltCallback(CAN_RxMessage->Data);               
            }
            break;
            case (0x24):
            {						
                // 500HZ	            
               chariot.Chassis.Joint_Motor[3].motor.CAN_RxCpltCallback(CAN_RxMessage->Data);               
            }
            break;
        #endif 
        #ifdef GIMBAL
            case (0x205):
            {
                chariot.Gimbal.Motor_Pitch.CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;
            case (0x141):
            {
                chariot.Gimbal.Motor_Pitch_LK6010.CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;
        #endif

        #ifdef BOOSTER
            case (0x203):
            {
                chariot.Booster.Motor_Driver.CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;
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
        #endif
        }

    }
    else
    {
        switch (CAN_RxMessage->Header.ExtId)
        {
        #ifdef CHASSIS
        case (0x2911):
        {
            // 500HZ
            chariot.Chassis.Wheel_Motor[1].motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        default:
        break;
        #endif
        }
    }
}

/**
 * @brief CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
uint8_t test_yaw;
void Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
	if(CAN_RxMessage->Header.IDE== CAN_ID_STD)
    {
      switch (CAN_RxMessage->Header.StdId)
    {
        #ifdef CHASSIS
            case (0x208):  //留给yaw电机编码器回传 用于底盘随动
            {
               
            }
            break;
            case (0x77):  //留给上板通讯
            {
                chariot.Information_Platform.Platform_Rx_Callback();
            }
            break;
            case (0x21):
            {	
                // 500HZ						            
               chariot.Chassis.Joint_Motor[0].motor.CAN_RxCpltCallback(CAN_RxMessage->Data);               
            }
            break;
			case (0x22):
            {			
                // 500HZ				            
               chariot.Chassis.Joint_Motor[1].motor.CAN_RxCpltCallback(CAN_RxMessage->Data);               
            }
            case (0x206):  //留给超级电容
            {
                test_yaw++;
            // chariot.Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;
        #endif 
        #ifdef GIMBAL
           
            case (0x88):   //留给下板通讯
            {
                chariot.Information_Platform.Platform_Rx_Callback();
            }
            break;
            case (0x208):   //保留can2对6020编码器的接口
            {
                chariot.Gimbal.Motor_Yaw.CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;
            case (0x204):
            {
        
            }
            break;
            case (0x205):
            {
       
            }
            break;
            case (0x206):
            {
        
            }
            break;
	        
        #endif

        #ifdef BOOSTER
           
        #endif
    }
	}
    else
    {
        switch (CAN_RxMessage->Header.ExtId)
        {
        #ifdef CHASSIS
            case (0x2912):
            {
                // 500HZ
                chariot.Chassis.Wheel_Motor[0].motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;
            default:
            break;
        #endif
        }
    }
}


/**
 * @brief SPI5回调函数
 *
 * @param Tx_Buffer SPI5发送的消息
 * @param Rx_Buffer SPI5接收的消息
 * @param Length 长度
 */
//void Device_SPI5_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
//{
//    if (SPI5_Manage_Object.Now_GPIOx == BoardA_MPU6500_CS_GPIO_Port && SPI5_Manage_Object.Now_GPIO_Pin == BoardA_MPU6500_CS_Pin)
//    {
//        boarda_mpu.SPI_TxRxCpltCallback(Tx_Buffer, Rx_Buffer);
//    }
//}

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
 * @brief UART1图传回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
void Image_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.DR16.Image_UART_RxCpltCallback(Buffer);

    //底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();
	
}



/**
 * @brief UART3遥控器回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */

void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{

    chariot.DR16.DR16_UART_RxCpltCallback(Buffer);

    //底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();
		
	
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
#ifdef REFEREE
void Referee_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.Referee.UART_RxCpltCallback(Buffer,Length);
}
#endif
/**
 * @brief UART1超电回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
#ifdef CHASSIS
void SuperCAP_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    // chariot.Chassis.Supercap.UART_RxCpltCallback(Buffer);
}
#endif
/**
 * @brief USB MiniPC回调函数
 *
 * @param Buffer USB收到的消息
 *
 * @param Length 长度
 */
#ifdef MINI_PC
void MiniPC_USB_Callback(uint8_t *Buffer, uint32_t Length)
{
    chariot.MiniPC.USB_RxCpltCallback(Buffer);
}
#endif
/**
 * @brief TIM4任务回调函数
 *
 */
void Task100us_TIM4_Callback()
{
        // 单给IMU消息开的定时器 ims
        #if defined(C_IMU)&&defined(GIMBAL)
        chariot.Gimbal.Boardc_BMI.TIM_Calculate_PeriodElapsedCallback();
        #endif
        #if defined(C_IMU)&&defined(CHASSIS)
        chariot.Chassis.Boardc_BMI.TIM_Calculate_PeriodElapsedCallback();
        #endif
}



/**
 * @brief TIM5任务回调函数
 *
 */
static uint8_t DM_Enable_Control_Index = 0;
void Task1ms_TIM5_Callback()
{
	init_finished++;
	if(init_finished>1000)
	start_flag=1;

	/************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/
	
	chariot.TIM1msMod50_Alive_PeriodElapsedCallback();
	
	/****************************** 交互层回调函数 1ms *****************************************/
	if(start_flag==1)
	{
			
		static int mod10 = 0;
		mod10+=1;
		
		chariot.FSM_Alive_Control.Reload_TIM_Status_PeriodElapsedCallback();
		chariot.Chassis.Observe.TIM_Calculate_PeriodElapsedCallback(1);
		
        // if(mod10%2==0)
        chariot.TIM_Calculate_PeriodElapsedCallback();
				
	/****************************** 驱动层回调函数 1ms *****************************************/ 
	    //统一打包发送
        TIM_CAN_PeriodElapsedCallback();
//        TIM_UART_PeriodElapsedCallback();	
        if (mod10 == 10)
        {
            TIM_USB_PeriodElapsedCallback(&MiniPC_USB_Manage_Object);   
            mod10 = 0;
        }	        
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
	
    #ifdef CAN
         //集中总线can1/can2
        CAN_Init(&hcan1, Device_CAN1_Callback);
        CAN_Init(&hcan2, Device_CAN2_Callback);
    #endif
       
    #ifdef UART
        //裁判系统,并未使用环形队列 尽量给长范围增加检索时间 减少丢包
        #ifdef REFEREE
        UART_Init(&huart6, Referee_UART6_Callback, 128);
				UART_Init(&huart6, Image_UART6_Callback, 40);
        //旧版超电
        //UART_Init(&huart1, SuperCAP_UART1_Callback, 128);
				#endif
        //遥控器接收
        UART_Init(&huart3, DR16_UART3_Callback, 18);

		
    #endif 

    #ifdef SPI
        //c板陀螺仪spi外设
        SPI_Init(&hspi1,Device_SPI1_Callback);

    #endif  

    #ifdef IIC
        //磁力计iic外设
        IIC_Init(&hi2c3, Ist8310_IIC3_Callback);    
    #endif

    #ifdef USB
        //上位机USB
        USB_Init(&MiniPC_USB_Manage_Object,MiniPC_USB_Callback);
    #endif  

    #ifdef PWM
        //舵机
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    #endif     

    // //定时器循环任务
    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    /********************************* 设备层初始化 *********************************/

    //设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/

    chariot.Init();

    /********************************* 使能调度时钟 *********************************/

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
}

/**
 * @brief 前台循环任务
 *
 */
 extern "C" void Task_Loop()
{


}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
