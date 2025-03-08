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
#include "config.h"

#ifdef GIMBAL_TASK
/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "drv_tim.h"
#include "CharSendTask.h"
#include "GraphicsSendTask.h"
#include "drv_usb.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

//#include "GraphicsSendTask.h"
//#include "ui.h"
#include "dvc_GraphicsSendTask.h"
#include "robotarm_task.h"
#include "dvc_message.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

extern Class_Robotarm Robotarm;

extern Struct_USB_Manage_Object MiniPC_USB_Manage_Object;

Struct_Offline_Controller_Data Offline_Controller_Data;

//Struct_Customize_Controller_Data Customize_Controller_Data;
//注册发布者
//Publisher UART1_Controller_Data = Message_Manager.PubRegister("Controller_Data", sizeof(Struct_Offline_Controller_Data));

/* Private function declarations ---------------------------------------------*/
float tmp_1_5_angle[5] = {2.0f,175.0f,180.0f,0.0f,90.0f};
float a=0.15;

/* Function prototypes -------------------------------------------------------*/


/**
 * @brief CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
	uint32_t temp_id;
	if(CAN_RxMessage->Header.IDE == CAN_ID_STD)
	{
		temp_id = CAN_RxMessage->Header.StdId;
        switch (temp_id)
        {
            case (0x203):
            { 
               Robotarm.Motor_Joint4.CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;
            case (0x202):
            {
               Robotarm.Motor_Joint5.CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;
						  case (0x05)://
            {
                Robotarm.Motor_Joint3.CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;
        }
	}
	else if(CAN_RxMessage->Header.IDE == CAN_ID_EXT)
	{
		temp_id = CAN_RxMessage->Header.ExtId &0xff;
        switch (temp_id)
        {
            case (0x04):
            {
                Robotarm.Motor_Joint2.CAN_RxCpltCallback(CAN_RxMessage->Data);
            }
            break;

          
          
				
		}
	}
	
}


void Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
	uint32_t temp_id;
	if(CAN_RxMessage->Header.IDE == CAN_ID_STD)
	{
		temp_id = CAN_RxMessage->Header.StdId;
	}
	else if(CAN_RxMessage->Header.IDE == CAN_ID_EXT)
	{
		temp_id = CAN_RxMessage->Header.ExtId &0xff;
	}
	
    switch (temp_id)
    {
        
        case 0x203:
        {
            Robotarm.Arm_Uplift.CAN_RxCpltCallback(CAN_RxMessage->Data);
					
        }
        break;
				 case 0x01:
            {
            Robotarm.Motor_Joint1.CAN_RxCpltCallback(CAN_RxMessage->Data);
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

void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
    Robotarm.DR16.DR16_UART_RxCpltCallback(Buffer);
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
    Robotarm.Referee.UART_RxCpltCallback(Buffer, Length);
}


/**
 * @brief 离线自定义扩展器回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */
void UART1_Offline_Controller_Callback(uint8_t *Buffer, uint16_t Length)
{
    if(Buffer[0]==0xA5 && Buffer[11]==0x11)
    {
        for(uint8_t i=0; i<5; i++)
        {
            int16_t temp = (Buffer[2*i+2]<<8) | Buffer[2*i+1];
            Offline_Controller_Data.Angle[i] = temp/100.f;
        }
    }
}
/**
 * @brief UART1图传回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
void Image_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    Robotarm.DR16.Image_UART_RxCpltCallback(Buffer);
//    for(uint8_t i=0; i<5; i++)
//    {
//        Customize_Controller_Data.Angle[i] = Robotarm.DR16.Get_Angle_Image(i);
//    }

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
    Robotarm.MiniPc.USB_RxCpltCallback(Buffer);
}

/**
 * @brief TIM4任务回调函数
 *
 */
void Task100us_TIM4_Callback()
{
    // 单给IMU消息开的定时器 ims
    // Robotarm.Boardc_BMI.TIM_Calculate_PeriodElapsedCallback();     
}

/**
 * @brief TIM5任务回调函数
 *
 */
#define DEBUG_REMOTE
#define IMAGE_REMOTE
//#define OFFLINE_REMOTE
#define DEBUG_REMOTE_SPEED_1 1.f
#define DEBUG_REMOTE_SPEED_2 1.f
//注册订阅者
Subscriber Task_Sub_Joint5_angle = Message_Manager.SubRegister("Joint5_angle", sizeof(float));
float debug_test_angle[5];
float debug_test_k = 1.0f;

void Set_Joint_1_5_Angle_Init_Data()
{		
	 
    #ifdef OFFLINE_REMOTE
    for (auto  i = 0; i < 5; i++)
    {
        Robotarm.Jonit_AngleInit[i] = Offline_Controller_Data.Angle[i];
    }
    #endif
    #ifdef IMAGE_REMOTE
    for (auto  i = 0; i < 5; i++)
    {
        Robotarm.Jonit_AngleInit[i] = Robotarm.DR16.Customize_Controller_Data.Angle[i];
    }
	//	Robotarm.Jonit_AngleInit[4]=Robotarm.Jonit_AngleInit[4]*2;
    #endif
   //  memcpy(Robotarm.Jonit_AngleInit, tmp_1_5_angle, 6 * sizeof(float));
    #ifdef DEBUG_REMOTE

    #endif
    if((Robotarm.Relay1.Get_Open_flag()==1)&&(Robotarm.DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN))
    Math_Constrain(Robotarm.Jonit_AngleInit[0], 2.f, 178.f);
    Math_Constrain(Robotarm.Jonit_AngleInit[1], 2.f, 178.f);
    Math_Constrain(Robotarm.Jonit_AngleInit[2], -25.f, 205.f);
    Math_Constrain(Robotarm.Jonit_AngleInit[3], 0.f, 180.f);
    Math_Constrain(Robotarm.Jonit_AngleInit[4], 0.f, 180.f);
}


void Task1ms_TIM5_Callback()
{
    /************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/
    static uint8_t TIM1msMod50 = 0;
	
    TIM1msMod50++;
    if (TIM1msMod50 == 50)
    {
        Robotarm.Task_Alive_PeriodElapsedCallback();//to do ：加入裁判系统
				Robotarm.Referee.UART_Tx_Referee_UI();
        TIM1msMod50 = 0;
    }
	
    // 遥控器控制
    switch (Robotarm.DR16.Get_DR16_Status())
    {
    case DR16_Status_ENABLE:
    {
        if (!Robotarm.init_finished)
			{ Robotarm.init_finished = Robotarm.Robotarm_Calibration();
			Robotarm.Robotarm_Resolution.Set_Status(Robotarm_Task_Status_Calibration);
			}
        else if (Robotarm.init_finished)
        {
            switch (Robotarm.DR16.Get_Right_Switch())
            {
            case (DR16_Switch_Status_DOWN): // 当遥控器右拨杆朝下的时候使能自定义控制器控制
            {			
									Robotarm.Motor_Joint4.PID_Angle.Set_K_P(18);
									Robotarm.Motor_Joint4.PID_Omega.Set_K_P(16);
									Robotarm.Motor_Joint5.PID_Angle.Set_K_P(18);
									Robotarm.Motor_Joint5.PID_Omega.Set_K_P(16);
							if(Robotarm.DR16.Get_Image_Status()==Image_Status_ENABLE)
							{Set_Joint_1_5_Angle_Init_Data();}
							else
							{  memcpy(Robotarm.Jonit_AngleInit, tmp_1_5_angle, 6 * sizeof(float));}	
									if (huart1.ErrorCode)
                                    {
										#ifdef OFFLINE_REMOTE
										UART_Init(&huart1, UART1_Offline_Controller_Callback, 12);//如果串口错误，重启使能串口
								    #endif
										#ifdef IMAGE_REMOTE
										UART_Init(&huart1, Image_UART1_Callback, 40);//如果串口错误，重启使能串口
										#endif
										}
									if(Robotarm.DR16.Get_Left_Switch()==DR16_Switch_Status_UP)//抬升高度控制，在自定义控制器控制模式下通过左拨杆控制抬升高度
									{Robotarm.Arm_Uplift.Target_Up_Length+=0.0025;}
									else if(Robotarm.DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN)
									{Robotarm.Arm_Uplift.Target_Up_Length-=0.0025;}
									if(Robotarm.DR16.Get_Left_Switch()==DR16_Switch_Status_MIDDLE)
  									{Robotarm.Arm_Uplift.Target_Up_Length=Robotarm.Arm_Uplift.Actual_Up_Length;}
									
									Math_Constrain(Robotarm.Arm_Uplift.Target_Up_Length,0.f,28.f);
										
						}
            break;
            case (DR16_Switch_Status_UP): // 当遥控器右拨杆朝上的时候使能miniPC控制
            {
//                if (Robotarm.MiniPc.Get_MiniPC_Status() == MiniPC_Status_DISABLE)
//                    memcpy(Robotarm.Jonit_AngleInit, tmp_1_5_angle, 5 * sizeof(float));
//                else
//                    Robotarm.MiniPc.Transform_Angle_Rx(Robotarm.Jonit_AngleInit);
//							
									Robotarm.Motor_Joint4.PID_Angle.Set_K_P(22);
									Robotarm.Motor_Joint4.PID_Omega.Set_K_P(20);
									Robotarm.Motor_Joint5.PID_Angle.Set_K_P(22);
									Robotarm.Motor_Joint5.PID_Omega.Set_K_P(20);
									Robotarm.Robotarm_Resolution.Reload_Task_Status_PeriodElapsedCallback();//状态机调试入口
							

            }
            break;
            case (DR16_Switch_Status_MIDDLE)://右拨杆为中时保持行进姿态
            {
                Robotarm.Robotarm_Resolution.Set_Status(Robotarm_Task_Status_Wait_Order);
                 memcpy(Robotarm.Jonit_AngleInit, tmp_1_5_angle, 6 * sizeof(float));
                Robotarm.Arm_Uplift.Target_Up_Length=0;
            }
            break;
            default:
            {
             memcpy(Robotarm.Jonit_AngleInit, tmp_1_5_angle, 6 * sizeof(float));
             Robotarm.Arm_Uplift.Target_Up_Length=0;
            }
            break;
            }
					if(Robotarm.DR16.Get_Right_Y()>0.8)//气泵继电器临时控制
                    {Robotarm.Relay1.Set_Open_flag(1);}
                    else if(Robotarm.DR16.Get_Right_Y()<-0.8)
                    {Robotarm.Relay1.Set_Open_flag(0);}
					if(Robotarm.Relay1.Get_Open_flag()==1)
					{HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);}
					else 
					{HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);}
						
						
						
			Robotarm.Output();
		} 
	}
    break;
    case DR16_Status_DISABLE:
    {
        Robotarm.TIM_Robotarm_Disable_PeriodElapsedCallback();
        Robotarm.init_finished = false;
    }
     break;
	}
    Robotarm.Boardc_BMI.TIM_Calculate_PeriodElapsedCallback();
		
    // 控制底盘任务
	  Robotarm.Judge_DR16_Control_Type();
    Robotarm.Control_Chassis_Task();
    Robotarm.CAN_Gimbal_Tx_Chassis();
    // 机械臂任务善后函数
    Robotarm.TIM_Robotarm_Task_PeriodElapsedCallback();
    // 发送CAN数据
    TIM_CAN_PeriodElapsedCallback();

    // 订阅者获取消息
    // Message_Manager.SubGet_Message<float>(Task_Sub_Joint5_angle, joint5_test_angle);

    /****************************** 交互层回调函数 1ms *****************************************/

    /****************************** 驱动层回调函数 1ms *****************************************/
    // 统一打包发送
    //    TIM_USB_PeriodElapsedCallback(&MiniPC_USB_Manage_Object);


}
/**
 * @brief 初始化任务
 *
 */

extern "C" void Task_Init()
{  

    DWT_Init(168);
    USB_Init(&MiniPC_USB_Manage_Object, MiniPC_USB_Callback);
    /********************************** 驱动层初始化 **********************************/

    CAN_Init(&hcan1, Device_CAN1_Callback);
    CAN_Init(&hcan2, Device_CAN2_Callback);
 
//    //c板陀螺仪spi外设
    SPI_Init(&hspi1,Device_SPI1_Callback);

//    //磁力计iic外设
     IIC_Init(&hi2c3, Ist8310_IIC3_Callback);    
	
  
    //裁判系统
    UART_Init(&huart6, Referee_UART6_Callback, 128);   //并未使用环形队列 尽量给长范围增加检索时间 减少丢包
    //遥控器接收
    UART_Init(&huart3, DR16_UART3_Callback, 18);
		
    //UART_Init(&huart1, Image_UART1_Callback, 40);
    //UART_Init(&huart1, UART1_Offline_Controller_Callback, 12);
	#ifdef OFFLINE_REMOTE
	UART_Init(&huart1, UART1_Offline_Controller_Callback, 12);//如果串口错误，重启使能串口
	#endif
	#ifdef IMAGE_REMOTE
	UART_Init(&huart1, Image_UART1_Callback, 40);//如果串口错误，重启使能串口
	#endif

    //定时器循环任务
    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);
		HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);

    /********************************* 设备层初始化 *********************************/

    //设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/

    Robotarm.Init();

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
#endif
#ifdef CHASSIS_TASK
#include "drv_tim.h"
#include "drv_can.h"
#include "task_callback_and_config.h"
#include "ita_chariot.h"
Class_Chariot chariot;
uint32_t init_finished = 0;
uint8_t start_flag = 0;


void Chariot_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x201):
    {
       chariot.chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
       chariot.chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
       chariot.chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
       chariot.chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}
void Chariot_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x205):
    {
        
    }
    case (0x088):
    {
       chariot.CAN_Chassis_Rx_Gimbal_Callback();
			
    }
    break;
    break;
    }
}

uint8_t mod50_cnt = 0;
void Task1ms_TIM5_Callback()
{
    init_finished++;
		mod50_cnt++;
    if(init_finished>2000)
    start_flag=1;
		if(mod50_cnt%50==0)
		{
		 chariot.TIM1msMod50_Alive_PeriodElapsedCallback();
		 mod50_cnt = 0;
		}
		chariot.TIM_Chariot_PeriodElapsedCallback();
		TIM_CAN_PeriodElapsedCallback();
}
extern "C" void Task_Init()
{
    //can总线
    CAN_Init(&hcan1,Chariot_Device_CAN1_Callback);
    CAN_Init(&hcan2,Chariot_Device_CAN2_Callback);
    //定时器循环任务
    TIM_Init(&htim5,Task1ms_TIM5_Callback);
    //交互层
    chariot.Init();
	
		HAL_TIM_Base_Start_IT(&htim5);
}
#endif

