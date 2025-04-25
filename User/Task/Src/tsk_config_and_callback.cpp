/**
 * @file tsk_config_and_callback.cpp
 * @author cjw by yssickjgd
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
#include "ita_chariot.h"
#include "dvc_imu.h"
#include "drv_usb.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "config.h"
//#include "GraphicsSendTask.h"
//#include "ui.h"
//#include "dvc_GraphicsSendTask.h"
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
 * @brief Chassis_CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
#ifdef CHASSIS
float temp[4];
uint16_t tmp[4];
float f_temp[4];
void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
        #ifdef AGV
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
        #endif
        #ifdef OMNI_WHEEL
            case (0x201):
            {
                chariot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;
            case (0x202):
            {
                chariot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;
            case (0x203):
            {
                chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;
            case (0x204):
            {
                chariot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;
        #endif
        case (0x150):  
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
    switch (CAN_RxMessage->Header.Identifier)
    {

        case (0x208):
        {
            chariot.Chassis.Motor_Steer[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x206):
        {
            chariot.Chassis.Motor_Steer[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x207):
        {
            chariot.Chassis.Motor_Steer[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x205):
        {
            chariot.Chassis.Motor_Steer[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;

    }
}
#endif

#ifdef CHASSIS
void Chassis_Device_CAN3_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage){
    switch (CAN_RxMessage->Header.Identifier)
    {

        case (0x77)://留给上板通讯
        {
            chariot.CAN_Chassis_Rx_Gimbal_Callback(CAN_RxMessage->Data);
            break;
        }
        case (0x67)://超电接收
        {
            chariot.Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case (0x55):
        {
            chariot.Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
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
    switch (CAN_RxMessage->Header.Identifier)
    {
        case (0x208):
        {
            chariot.Booster_B.Motor_Friction_Left.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x207):
        {
            chariot.Booster_B.Motor_Friction_Right.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x205):
        {
            chariot.Gimbal.Motor_Yaw_B.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x206):
        {
            chariot.Gimbal.Motor_Pitch_B.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
	}
}
#endif

/**
 * @brief Gimbal_CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
#ifdef GIMBAL
void Gimbal_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
        case (0x207):
        {
            chariot.Booster_A.Motor_Friction_Left.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x208):
        {
            chariot.Booster_A.Motor_Friction_Right.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x206):
        {
            chariot.Gimbal.Motor_Pitch_A.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x205):
        {
            chariot.Gimbal.Motor_Yaw_A.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;        
    }
		
}
#endif

#ifdef GIMBAL
void Gimbal_Device_CAN3_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage){
    switch (CAN_RxMessage->Header.Identifier)
    {
        case (0x141):
        {
            chariot.Gimbal.Motor_Main_Yaw.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x202):
        {
            chariot.Booster_A.Motor_Driver.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x203):
        {
            chariot.Booster_B.Motor_Driver.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x88):   //留给下板通讯
        {
            chariot.CAN_Gimbal_Rx_Chassis_Callback();
        }
        break;
        case (0x78):
        {
            chariot.CAN_Gimbal_Rx_Chassis_Callback();
        }
        break;
        case (0x99):
        {
            chariot.CAN_Gimbal_Rx_Chassis_Callback();
        }
        break;
        case (0x98):
        {
            chariot.CAN_Gimbal_Rx_Chassis_Callback();
        }
        break;
        case (0x104):
        {
            chariot.MiniPC.CAN_RxCpltCallback();
        }
        break;
        case (0x105):
        {
            chariot.MiniPC.CAN_RxCpltCallback();
        }
        break;
        case (0x106):
        {
            chariot.MiniPC.CAN_RxCpltCallback();
        }
        break;
        case (0x107):
        {
            chariot.MiniPC.CAN_RxCpltCallback();
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
void Device_SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
{

}

/**
 * @brief UART1陀螺仪回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
#ifdef GIMBAL
void IMUA_UART7_Callback(uint8_t *Buffer, uint16_t Length)
{
    uint8_t Gyro_Data[28];
    int i = 0,j = 0;
    for(i = 0; i < 28; i++)
    {
        if((Buffer[i] == 0x55 && Buffer[i+1] == 0x55) && Buffer[i+2] == 0x01)
        {
            for(j = 0; j < 28; j++)
            {
                Gyro_Data[j] = Buffer[i];
                i++;
                if(i >= 29)
                {
                    Gyro_Data[j] = Buffer[i-29];
                }
            }
            break;
        }
    }
    chariot.Gimbal.IMU_Data_A.Roll = (float)((int16_t)(Gyro_Data[5] << 8) | Gyro_Data[4]) / 32768.0f * 180.0f;
    chariot.Gimbal.IMU_Data_A.Pitch = (float)((int16_t)(Gyro_Data[7] << 8) | Gyro_Data[6]) / 32768.0f * 180.0f;
    chariot.Gimbal.IMU_Data_A.Yaw = (float)((int16_t)(Gyro_Data[9] << 8) | Gyro_Data[8]) / 32768.0f * 180.0f;
    chariot.Gimbal.IMU_Data_A.Omega_X = (float)((int16_t)(Gyro_Data[22] << 8) | Gyro_Data[21]) / 32768.0f * 2000.0f;
    chariot.Gimbal.IMU_Data_A.Omega_Y = (float)((int16_t)(Gyro_Data[24] << 8) | Gyro_Data[23]) / 32768.0f * 2000.0f;
    chariot.Gimbal.IMU_Data_A.Omega_Z = (float)((int16_t)(Gyro_Data[26] << 8) | Gyro_Data[25]) / 32768.0f * 2000.0f;
}
#endif


/**
 * @brief UART7陀螺仪回调函数
 *
 * @param Buffer UART7收到的消息
 * @param Length 长度
 */

#ifdef GIMBAL
void IMUB_USART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    uint8_t Gyro_Data[28]; 
    int i = 0,j = 0;
    for(i = 0; i < 28; i++)
    {
        if((Buffer[i] == 0x55 && Buffer[i+1] == 0x55) && Buffer[i+2] == 0x01)
        {
            for(j = 0; j < 28; j++)
            {
                Gyro_Data[j] = Buffer[i];
                i++;
                if(i >= 29)
                {
                    Gyro_Data[j] = Buffer[i-29];
                }
            }
            break;
        }
    }
    chariot.Gimbal.IMU_Data_B.Roll = (float)((int16_t)(Gyro_Data[5] << 8) | Gyro_Data[4]) / 32768.0f * 180.0f;
    chariot.Gimbal.IMU_Data_B.Pitch = (float)((int16_t)(Gyro_Data[7] << 8) | Gyro_Data[6]) / 32768.0f * 180.0f;
    chariot.Gimbal.IMU_Data_B.Yaw = (float)((int16_t)(Gyro_Data[9] << 8) | Gyro_Data[8]) / 32768.0f * 180.0f;
    chariot.Gimbal.IMU_Data_B.Omega_X = (float)((int16_t)(Gyro_Data[22] << 8) | Gyro_Data[21]) / 32768.0f * 2000.0f;
    chariot.Gimbal.IMU_Data_B.Omega_Y = (float)((int16_t)(Gyro_Data[24] << 8) | Gyro_Data[23]) / 32768.0f * 2000.0f;
    chariot.Gimbal.IMU_Data_B.Omega_Z = (float)((int16_t)(Gyro_Data[26] << 8) | Gyro_Data[25]) / 32768.0f * 2000.0f;
}
#endif


/**
 * @brief UART5遥控器回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
#ifdef GIMBAL
void DR16_UART5_Callback(uint8_t *Buffer, uint16_t Length)
{

    chariot.DR16.DR16_UART_RxCpltCallback(Buffer);

    //底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();
		
}
#endif

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
#ifdef CHASSIS
void Referee_UART10_Callback(uint8_t *Buffer, uint16_t Length)
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
#if defined CHASSIS
void SuperCAP_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.Chassis.Supercap.UART_RxCpltCallback(Buffer);
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
 * @brief UART MiniPC回调函数
 *
 * @param Buffer UART收到的消息
 *
 * @param Length 长度
 */
#ifdef GIMBAL
void MiniPC_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.MiniPC.UART_RxCpltCallback(Buffer);
}
#endif
/**
 * @brief TIM4任务回调函数
 *
 */
void Task100us_TIM4_Callback()
{
    #ifdef CHASSIS
        // static uint16_t Referee_Sand_Cnt = 0;
        // //暂无云台tim4任务
        // if(Referee_Sand_Cnt%10)
             //Task_Loop();

    #elif defined(GIMBAL)
        // 单给IMU消息开的定时器 ims
        chariot.Gimbal.Boardc_BMI.TIM_Calculate_PeriodElapsedCallback();     
        IMUA_UART7_Callback(UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Length);
        IMUB_USART1_Callback(UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Length);
    #endif
}



/**
 * @brief TIM5任务回调函数
 *
 */
extern Referee_Rx_A_t CAN3_Chassis_Rx_Data_A;
void Task1ms_TIM5_Callback()
{
    init_finished++;
    if(init_finished>2000)
    start_flag=1;

    /************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/
    
    chariot.TIM1msMod50_Alive_PeriodElapsedCallback();

    /****************************** 交互层回调函数 1ms *****************************************/
    if(start_flag==1)
    {
        #ifdef GIMBAL
            #ifdef DEBUG
            chariot.FSM_Alive_Control.Reload_TIM_Status_PeriodElapsedCallback();
            #else
            if(CAN3_Chassis_Rx_Data_A.game_process != 4)
            {
                chariot.FSM_Alive_Control.Reload_TIM_Status_PeriodElapsedCallback();
            }
            #endif
        #endif

        chariot.TIM_Calculate_PeriodElapsedCallback();
        
    /****************************** 驱动层回调函数 1ms *****************************************/ 
        //统一打包发送
        TIM_CAN_PeriodElapsedCallback();
        //上位机
        static int mod5 = 0;
        mod5++;
        if (mod5 == 5)
        {
            TIM_USB_PeriodElapsedCallback(&MiniPC_USB_Manage_Object);
            TIM_UART_PeriodElapsedCallback();
            mod5 = 0;
        }	        

    }
}

/**
 * @brief 初始化任务
 *
 */
extern "C" void Task_Init()
{  

    DWT_Init(480);

    /********************************** 驱动层初始化 **********************************/
	#ifdef CHASSIS

        //集中总线can1/can2
        CAN_Init(&hfdcan1, Chassis_Device_CAN1_Callback);
        CAN_Init(&hfdcan2, Chassis_Device_CAN2_Callback);
        CAN_Init(&hfdcan3, Chassis_Device_CAN3_Callback);

        //裁判系统
        UART_Init(&huart10, Referee_UART10_Callback, 128);//并未使用环形队列 尽量给长范围增加检索时间 减少丢包

        #ifdef POWER_LIMIT


        #endif

    #endif

    #ifdef GIMBAL

        //集中总线can1/can2
        CAN_Init(&hfdcan1, Gimbal_Device_CAN1_Callback);
        CAN_Init(&hfdcan2, Gimbal_Device_CAN2_Callback);
        CAN_Init(&hfdcan3, Gimbal_Device_CAN3_Callback);

        //c板陀螺仪spi外设
        SPI_Init(&hspi2,Device_SPI2_Callback);
        //磁力计iic外设
        //IIC_Init(&hi2c3, Ist8310_IIC3_Callback);    //达妙无磁力计
        //遥控器接收
        UART_Init(&huart5, DR16_UART5_Callback, 18);
        //陀螺仪
		UART_Init(&huart7, IMUA_UART7_Callback, 56);
        UART_Init(&huart1, IMUB_USART1_Callback, 56);
        //上位机USB
        USB_Init(&MiniPC_USB_Manage_Object,MiniPC_USB_Callback);
        //上位机串口
        UART_Init(&huart8, MiniPC_UART_Callback, 56);

    #endif

    //定时器循环任务
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
    #ifdef GIMBAL

    #endif
    #ifdef CHASSIS

    #endif
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
