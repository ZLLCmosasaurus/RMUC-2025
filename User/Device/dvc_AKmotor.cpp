/**
 * @file dvc_AKmotor.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief AK电机配置与操作
 * @version 0.1
 * @date 2023-08-30 0.1 初稿机
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_AKmotor.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

//使能电机
uint8_t AK_Motor_CAN_Message_Enter[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
//失能电机
uint8_t AK_Motor_CAN_Message_Exit[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
//保存当前电机位置为零点
uint8_t AK_Motor_CAN_Message_Save_Zero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, Enum_AK_Motor_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hcan1)
    {
        switch (__CAN_ID)
        {
        case (AK_Motor_ID_0x01):
        {
            tmp_tx_data_ptr = CAN1_0xx01_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x02):
        {
            tmp_tx_data_ptr = CAN1_0xx02_Tx_Data ;
        }
        break;
        case (AK_Motor_ID_0x03):
        {
            tmp_tx_data_ptr = CAN1_0xx03_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x04):
        {
            tmp_tx_data_ptr = CAN1_0xx04_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x05):
        {
            tmp_tx_data_ptr = CAN1_0xx05_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x06):
        {
            tmp_tx_data_ptr = CAN1_0xx06_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x07):
        {
            tmp_tx_data_ptr = CAN1_0xx07_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x08):
        {
            tmp_tx_data_ptr = CAN1_0xx08_Tx_Data;
        }
        break;
        }
    }
    else if (hcan == &hcan2)
    {
        switch (__CAN_ID)
        {
        case (AK_Motor_ID_0x01):
        {
            tmp_tx_data_ptr = CAN2_0xx01_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x02):
        {
            tmp_tx_data_ptr = CAN2_0xx02_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x03):
        {
            tmp_tx_data_ptr = CAN2_0xx03_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x04):
        {
            tmp_tx_data_ptr = CAN2_0xx04_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x05):
        {
            tmp_tx_data_ptr = CAN2_0xx05_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x06):
        {
            tmp_tx_data_ptr = CAN2_0xx06_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x07):
        {
            tmp_tx_data_ptr = CAN2_0xx07_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x08):
        {
            tmp_tx_data_ptr = CAN2_0xx08_Tx_Data;
        }
        break;
        }
    }
    return (tmp_tx_data_ptr);
}

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_ID 绑定的CAN ID
 * @param __Control_Method 电机控制方式, 默认角度
 * @param __Position_Offset 编码器偏移, 默认0
 * @param __Omega_Max 最大速度, 调参助手设置
 * @param __Torque_Max 最大扭矩, 调参助手设置
 */
void Class_AK_Motor_80_6::Init(CAN_HandleTypeDef *hcan, Enum_AK_Motor_ID __CAN_ID, Enum_AK_Motor_Control_Method __Control_Method, float __MIT_K_P,float __MIT_K_D,
								int32_t __Position_Offset, float __Angle_Max,float __Omega_Max, float __Torque_Max,float __Slope_Angle)
{
    if (hcan->Instance == CAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    CAN_ID = __CAN_ID;
    AK_Motor_Control_Method = __Control_Method;
	MIT_K_P = __MIT_K_P;
	MIT_K_D = __MIT_K_D;
    Position_Offset = __Position_Offset;
	Angle_Max = __Angle_Max;
    Omega_Max = __Omega_Max;
    Torque_Max = __Torque_Max;
	//斜坡函数加减速速度X
    Slope_Joint_Angle.Init(__Slope_Angle, __Slope_Angle);
    CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);
}

/**
 * @brief 数据处理过程
 *
 */
void Class_AK_Motor_80_6::Data_Process()
{
	if(AK_Motor_Control_Method ==CAN_PACKET_SET_RUN_CONTROL)
	{
		//数据处理过程
		int32_t delta_position;
		int16_t tmp_position, tmp_omega, tmp_torque;
		Struct_AK_Motor_RUN_CAN_Rx_Data *tmp_buffer = (Struct_AK_Motor_RUN_CAN_Rx_Data *)CAN_Manage_Object->Rx_Buffer.Data;

		//处理大小端
		Math_Endian_Reverse_16((void *)&tmp_buffer->Position_Reverse, &tmp_position);
		tmp_omega = (tmp_buffer->Omega_11_4 << 4) | (tmp_buffer->Omega_3_0_Torque_11_8 >> 4);
		tmp_torque = ((tmp_buffer->Omega_3_0_Torque_11_8 & 0x0f) << 8) | (tmp_buffer->Torque_7_0);

		Data.CAN_ID = tmp_buffer->CAN_ID;

		//计算圈数与总角度值
		delta_position = tmp_position - Data.Pre_Position;
		if (delta_position < -(Position_Max / 2))
		{
			//正方向转过了一圈
			Data.Total_Round++;
		}
		else if (delta_position > (Position_Max / 2))
		{
			//反方向转过了一圈
			Data.Total_Round--;
		}
		Data.Total_Position = Data.Total_Round * Position_Max + tmp_position + Position_Offset;

		//计算电机本身信息
	//    Data.Now_Angle = (float)Data.Total_Position / (float)Position_Max * 3;
		Data.Now_Angle = Math_Int_To_Float(tmp_position, 0, (1 << 16) - 1, -Angle_Max, Angle_Max)*RAD_TO_DEG;
		Data.Now_Omega = Math_Int_To_Float(tmp_omega, 0, (1 << 12) - 1, -Omega_Max, Omega_Max);
		Data.Now_Torque = Math_Int_To_Float(tmp_torque, 0, (1 << 12) - 1, -Torque_Max, Torque_Max);
		Data.Now_Rotor_Temperature = tmp_buffer->Motor_Temperature -40.0f ;

		//存储预备信息
		Data.Pre_Position = tmp_position;
	}
	else if(AK_Motor_Control_Method != CAN_PACKET_DIS_RUN_CONTROL)
	{
		//数据处理过程
		int32_t delta_position;
		int16_t tmp_position, tmp_omega, tmp_torque;
		float tmp_speed;
		Struct_AK_Motor_CAN_Rx_Data *tmp_buffer = (Struct_AK_Motor_CAN_Rx_Data *)CAN_Manage_Object->Rx_Buffer.Data;

		//处理大小端
		Math_Endian_Reverse_16((void *)&tmp_buffer->Position_Reverse, (void *)&tmp_position);
		Math_Endian_Reverse_16((void *)&tmp_buffer->Omega_Reverse, (void *)&tmp_omega);
		Math_Endian_Reverse_16((void *)&tmp_buffer->Torque_Reverse, (void *)&tmp_torque);

		Data.CAN_ID = static_cast<Enum_AK_Motor_ID>(CAN_Manage_Object->Rx_Buffer.Header.ExtId &0xff);

		//计算圈数与总角度值
		delta_position = tmp_position - Data.Pre_Position;
		if (delta_position < (int32_t)(-(Position_Max / 2)))
		{
			//正方向转过了一圈
			Data.Total_Round++;
		}
		else if (delta_position > (int32_t)((Position_Max / 2)))
		{
			//反方向转过了一圈
			Data.Total_Round--;
		}
		Data.Total_Position = Data.Total_Round * Position_Max + tmp_position + Position_Offset;

		//计算电机本身信息
		if(fabs(tmp_omega) < 32000.0f)
		{
			Data.Now_Angle = AK80_POSITION_FROM_LSB_TO_FLOAT(tmp_position);
			Data.Now_Omega = AK80_SPEED_FROM_LSB_TO_FLOAT(tmp_omega)*PI/180.f/21.f;
			Data.Now_Torque = AK80_CURRENT_FROM_LSB_TO_FLOAT(tmp_torque);
			Data.Now_Rotor_Temperature = AK80_TEMPERATURE_FROM_LSB_TO_FLOAT(tmp_buffer->Motor_Temperature);
			Data.error_statue = tmp_buffer->error_statue;
			//存储预备信息
			Data.Pre_Position = tmp_position;
		}
	}
    
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_AK_Motor_80_6::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_AK_Motor_80_6::Task_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过电机数据
    if ((Flag == Pre_Flag)||(Data.error_statue != 0))
    {
        //电机断开连接
        AK_Motor_Status = AK_Motor_Status_DISABLE;
    }
    else
    {
        //电机保持连接
        AK_Motor_Status = AK_Motor_Status_ENABLE;
    }

    //控制电机使能或失能
    // switch (AK_Motor_Control_Method)
    // {
	// 	case (CAN_PACKET_DIS_RUN_CONTROL):
	// 	{
	// 		CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_AK_Motor_ID>(CAN_ID), AK_Motor_CAN_Message_Exit, (uint16_t)8,CAN_ID_STD);
	// 	}
	// 	break;
	// 	case (CAN_PACKET_SET_RUN_CONTROL):
	// 	{
	// 		CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_AK_Motor_ID>(CAN_ID), AK_Motor_CAN_Message_Enter, (uint16_t)8,CAN_ID_STD);
	// 	}
	// 	break;
	// 	default:
	// 	break;
    // }
    Pre_Flag = Flag;
}


void Class_AK_Motor_80_6::Task_PID_PeriodElapsedCallback()
{
	switch(AK_Motor_Control_Method)
	{
		case(CAN_PACKET_SET_POS_SPD):
		{
			PID_Angle.Set_Target(Target_Angle);
			PID_Angle.Set_Now(Data.Now_Angle);
			PID_Angle.TIM_Adjust_PeriodElapsedCallback();
			Target_Omega = PID_Angle.Get_Out();
		}
		break;
		default:
		break;
	}
}

/**
 * @brief TIM定时器中断发送出去的回调函数
 *
 */

uint8_t can_flag;
void Class_AK_Motor_80_6::Task_Process_PeriodElapsedCallback()
{
	float tmp_angle;
	Slope_Joint_Angle.Set_Target(Target_Angle);
	Slope_Joint_Angle.TIM_Calculate_PeriodElapsedCallback();
	tmp_angle = Slope_Joint_Angle.Get_Out();
    switch (AK_Motor_Control_Method)
    {
	case (CAN_PACKET_SET_RUN_CONTROL):
    {
        uint16_t tmp_position = Math_Float_To_Int(tmp_angle*DEG_TO_RAD, -Angle_Max, Angle_Max, 0, (1 << 16) - 1);
        uint16_t tmp_velocity = Math_Float_To_Int(Target_Omega, -Omega_Max, Omega_Max, 0, (1 << 12) - 1);
        uint16_t tmp_k_p = Math_Float_To_Int(MIT_K_P, 0.0f, 500.0f, 0, (1 << 12) - 1);
        uint16_t tmp_k_d = Math_Float_To_Int(MIT_K_D, 0.0f, 5.0f, 0, (1 << 12) - 1);
        uint16_t tmp_torque = Math_Float_To_Int(Target_Torque, -Torque_Max, Torque_Max, 0, (1 << 12) - 1);

        Math_Endian_Reverse_16(&tmp_position);
        memcpy(&CAN_Tx_Data[0], &tmp_position, sizeof(uint16_t));

        uint8_t tmp_velocity_11_4 = tmp_velocity >> 4;
        memcpy(&CAN_Tx_Data[2], &tmp_velocity_11_4, sizeof(uint8_t));

        uint8_t tmp_velocity_3_0_k_p_11_8 = ((tmp_velocity & 0x0f) << 4) | (tmp_k_p >> 8);
        memcpy(&CAN_Tx_Data[3], &tmp_velocity_3_0_k_p_11_8, sizeof(uint8_t));

        uint8_t tmp_k_p_7_0 = tmp_k_p;
        memcpy(&CAN_Tx_Data[4], &tmp_k_p_7_0, sizeof(uint8_t));

        uint8_t tmp_k_d_11_4 = tmp_k_d >> 4;
        memcpy(&CAN_Tx_Data[5], &tmp_k_d_11_4, sizeof(uint8_t));

        uint8_t tmp_k_d_3_0_torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        memcpy(&CAN_Tx_Data[6], &tmp_k_d_3_0_torque_11_8, sizeof(uint8_t));

        uint8_t tmp_torque_7_0 = tmp_torque;
        memcpy(&CAN_Tx_Data[7], &tmp_torque_7_0, sizeof(uint8_t));
    }
    break;
    case (CAN_PACKET_SET_POS_SPD):
    {
        int32_t tmp_position = AK80_POSITION_FROM_FLOAT_TO_LSB(Target_Angle);
        int16_t tmp_velocity = AK80_SPEED_POSITION_SPD_FROM_FLOAT_TO_LSB(Target_Omega);
        int16_t tmp_torque = AK80_SPEED_POSITION_ACL_FROM_FLOAT_TO_LSB(Target_Torque);

        Math_Endian_Reverse_32(&tmp_position);
        memcpy(&CAN_Tx_Data[0], &tmp_position, sizeof(uint32_t));

        Math_Endian_Reverse_16(&tmp_velocity);
        memcpy(&CAN_Tx_Data[4], &tmp_velocity, sizeof(uint16_t));
			
		Math_Endian_Reverse_16(&tmp_torque);
        memcpy(&CAN_Tx_Data[6], &tmp_torque, sizeof(uint16_t));

//        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, (uint32_t)(AK_Motor_Control_Method<<8|CAN_ID), CAN_Tx_Data, 8, CAN_ID_EXT);
    }
    break;
	case (CAN_PACKET_SET_RPM):
    {
        int32_t tmp_velocity = AK80_SPEED_FROM_FLOAT_TO_LSB(Target_Omega);

        Math_Endian_Reverse_32(&tmp_velocity);
        memcpy(&CAN_Tx_Data[0], &tmp_velocity, sizeof(uint32_t));
			
//        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, (uint32_t)(AK_Motor_Control_Method<<8|CAN_ID), CAN_Tx_Data, 8, CAN_ID_EXT);
    }
    break;
    	case (CAN_PACKET_SET_CURRENT):
    {
        int32_t tmp_current = AK80_SPEED_FROM_FLOAT_TO_LSB(Target_Current*1000.f);

        Math_Endian_Reverse_32(&tmp_current);
        memcpy(CAN_Tx_Data, &tmp_current, sizeof(int32_t));
			
//        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, (uint32_t)(AK_Motor_Control_Method<<8|CAN_ID), CAN_Tx_Data, 8, CAN_ID_EXT);
    }
    break;
    default:
    {
    }
    break;
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
