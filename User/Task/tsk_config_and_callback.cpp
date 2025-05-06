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
#include "drv_usb.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "robotarm_task.h"
#include "dvc_message.h"
#include "ui.h"

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
						case (0x10)://
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
 * @brief UART1图传回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
void Image_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    Robotarm.DR16.Image_UART_RxCpltCallback(Buffer);
	
	
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
     Robotarm.Boardc_BMI.TIM_Calculate_PeriodElapsedCallback();     
}


//注册订阅者
//Subscriber Task_Sub_Joint5_angle = Message_Manager.SubRegister("Joint5_angle", sizeof(float));

uint16_t GetCCRFromAngle(float InputAngle){
float Ret=InputAngle/180 * 2000 +500;
return Ret;
}


/**
 * @brief TIM5任务回调函数
 *
 */
void Task1ms_TIM5_Callback()
{
    /************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/
    static uint8_t TIM1msMod50 = 0;
	  static uint8_t TIM1msMod100 = 0;
		TIM1msMod100++;
    TIM1msMod50++;
    if (TIM1msMod50 == 50)
    {
        Robotarm.Task_Alive_PeriodElapsedCallback();
        TIM1msMod50 = 0;
				Robotarm.Robotarm_Referee_UI_Tx_Callback(Robotarm.Referee_UI_Refresh_Status);
			
    }
		if((Robotarm.DR16.Get_DR16_Status()==DR16_Status_DISABLE)&&(Robotarm.DR16.Get_Image_Key_Status()==Image_Status_DISABLE))//遥控器与图传链路均失联//&&(Robotarm.DR16.Get_Image_Key_Status()==Image_Status_DISABLE)
		{		
		    Robotarm.TIM_Robotarm_Disable_PeriodElapsedCallback();
        Robotarm.init_finished = false;
				Robotarm.Chassis.Target_Yaw=-Robotarm.Boardc_BMI.Get_Angle_YawTotal();
				Robotarm.Arm_Uplift.Target_Up_Length=0;
				Robotarm.Gimbal_Control_Key_Type=Gimbal_Control_Type_Key_Normal;
		}
		else
		{
			if (!Robotarm.init_finished)
			{Robotarm.init_finished = Robotarm.Robotarm_Calibration();
			Robotarm.Robotarm_Resolution.Set_Status(Robotarm_Task_Status_Calibration);
			Robotarm.Gimbal_Control_Key_Type=Gimbal_Control_Type_Key_Normal;
			}
       else if (Robotarm.init_finished)
			{
			Robotarm.TIM_Control_Callback();	
			Robotarm.Output();
			} 
			Robotarm.Judge_DR16_Control_Type();// 判断DR16控制数据来源
			Robotarm.Control_Chassis_Task();//底盘控制   不校准完也可操纵底盘
		}
		
		
			if (huart1.ErrorCode)
			UART_Init(&huart1, Image_UART1_Callback, 40);//如果串口错误，重启使能串口

    // 机械臂任务善后函数
    Robotarm.TIM_Robotarm_Task_PeriodElapsedCallback();
		Robotarm.CAN_Gimbal_Tx_Chassis();
		//继电器任务善后函数
		Robotarm.Relays.Relay_Control();
    // 发送CAN数据
    TIM_CAN_PeriodElapsedCallback();
     //USB统一打包发送	
//		if (TIM1msMod100 == 100)
//    {
//        TIM1msMod100 = 0;
//			Robotarm.MiniPc.Output_en(Robotarm.Joint_World_Angle_Now,Robotarm.Arm_Uplift.Actual_Up_Length,Exchange_Status_ENABLE);
//			  TIM_USB_PeriodElapsedCallback(&MiniPC_USB_Manage_Object);

//    }
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,GetCCRFromAngle(Robotarm.servo_angle_picth));
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,GetCCRFromAngle(Robotarm.servo_angle_yaw));
   
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
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);

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
    case (0x205):
    {
        chariot.Auxiliary_Arm_Uplift_X_Right.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x206):
    {
        chariot.Auxiliary_Arm_Uplift_Y_Right.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
		 case (0x207):
    {
        chariot.Auxiliary_Arm_Uplift_X_Lift.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x208):
    {
        chariot.Auxiliary_Arm_Uplift_Y_Lift.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}
void Chariot_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
  

    case (0x088):
    {
       chariot.CAN_Chassis_Rx_Gimbal_Callback();
			
    }
    break;
    }
}
enum Enum_Robotarm_Task_Status
{
  	  	Robotarm_Task_Status_Calibration = 1,
		Robotarm_Task_Status_Wait_Order=2,
		Robotarm_Task_Status_Error=3,
		Robotarm_Task_Status_Exchange=4,
		Robotarm_Task_Status_Exchange_Pull=5,
		Robotarm_Task_Status_Assist_Calibration =6,
    Robotarm_Task_Status_Pick_Sliver_1=11,
		Robotarm_Task_Status_Pick_Sliver_2=12,
		Robotarm_Task_Status_Pick_Sliver_3=13,
		Robotarm_Task_Status_Pick_Sliver_3_1=33,
		Robotarm_Task_Status_Pick_Sliver_4=14,
		Robotarm_Task_Status_Pick_Sliver_5=15,
		Robotarm_Task_Status_Pick_Sliver_6=16,
		Robotarm_Task_Status_Pick_Sliver_6_1=36,
		Robotarm_Task_Status_Pick_Sliver_7=17,
		Robotarm_Task_Status_Pick_Sliver_8=18,
		Robotarm_Task_Status_Pick_Sliver_9=19,
		Robotarm_Task_Status_Pick_Gold_1=21,
		Robotarm_Task_Status_Pick_Gold_2=22,
		Robotarm_Task_Status_Pick_Gold_3=23,
		Robotarm_Task_Status_Pick_Gold_4=24,
		Robotarm_Task_Status_Pick_Gold_5=25,
		Robotarm_Task_Status_Pick_Gold_6=26,
		Robotarm_Task_Status_Pick_Gold_7=27,
	

};

uint16_t mod50_cnt = 0;
uint8_t mod50_cnt_2 = 0;
float angle_test;

void Task1ms_TIM5_Callback()
{
    if(chariot.init_finish_flag==false)
    {
         chariot.init_finish_flag=chariot.Motor_Calibration();
				 
    }
			mod50_cnt++;
			mod50_cnt_2++;
	
		if(chariot.init_finish_flag==true){
		if(chariot.Now_status==Robotarm_Task_Status_Assist_Calibration)
		{
			chariot.init_finish_flag=false;
		 chariot.Init();
	
		}
		switch(chariot.Now_status)
		{
			case(Robotarm_Task_Status_Calibration):
			{			
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=4;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-4;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=0.5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-0.5;
						
			}
			break;
			case(Robotarm_Task_Status_Pick_Gold_1):
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=1;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-1;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=0.5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-0.5;
						
			}
			break;
			case(Robotarm_Task_Status_Pick_Gold_2):
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=0.5;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-0.5;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=26.5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-26;
						
			}
			break;
			
			case(Robotarm_Task_Status_Pick_Gold_3):
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=3.2;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-3.2;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=26.5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-26;
						
			}
			break;
				case(Robotarm_Task_Status_Pick_Gold_4):
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=3.2;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-3.2;
				if(	(chariot.Auxiliary_Arm_Uplift_Y_Lift.Actual_Up_Length>2.7)&&(	chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length<-2.7))
				{chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=12;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-10;}
						
			}
			break;
				case(Robotarm_Task_Status_Pick_Gold_5):
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=1;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-1;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=0.5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-0.5;
						
			}
			break;
			case(Robotarm_Task_Status_Pick_Gold_6):
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=1;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-1;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=0.5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-0.5;
						
			}
			break;
			case(Robotarm_Task_Status_Exchange):
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=5.5;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-5.5;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=2;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-2;
						
			}
			break;
			case Robotarm_Task_Status_Exchange_Pull:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=1;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-1;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=23;//13
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-14;//13
			
			}
			break;
			case Robotarm_Task_Status_Wait_Order:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=4;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-4;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=0.5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-0.5;
			}
			break;
			case Robotarm_Task_Status_Pick_Sliver_1:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=1;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-1;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=0.5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-0.5;
			
			
			}break;
			case Robotarm_Task_Status_Pick_Sliver_2:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=1;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-1;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=0.5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-0.5;
			
			
			}break;
			case Robotarm_Task_Status_Pick_Sliver_3:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=1;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-1;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=0.5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-0.5;
			
			
			}break;
			case Robotarm_Task_Status_Pick_Sliver_3_1:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=6;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-2;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=23;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-0.5;
			
			
			}break;
			case Robotarm_Task_Status_Pick_Sliver_4:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=1;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-1;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-0.5;
			
			
			}break;
			case Robotarm_Task_Status_Pick_Sliver_5:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=1;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-2;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-0.5;
			
			
			}break;
			case Robotarm_Task_Status_Pick_Sliver_6:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=1;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-2;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-0.5;
			
			
			}break;
						case Robotarm_Task_Status_Pick_Sliver_6_1:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=1;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-6;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-14;
			
			
			}break;
						case Robotarm_Task_Status_Pick_Sliver_7:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=1;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-1;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=2;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-2;
			
			
			}break;
			case Robotarm_Task_Status_Pick_Sliver_8:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=1;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-1;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=2;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-2;
			
			
			}break;
			case Robotarm_Task_Status_Pick_Sliver_9:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=2;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-2;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=2;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-2;
			
			
			}break;
			default:
			{
						chariot.Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=3;
						chariot.Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-3;
						chariot.Auxiliary_Arm_Uplift_X_Lift.Target_Up_Length=0.5;
						chariot.Auxiliary_Arm_Uplift_X_Right.Target_Up_Length=-0.5;
						
			
			}
			break;
		}
		mod50_cnt_2=0;
	}
		
		

		if(mod50_cnt%500==0)
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

