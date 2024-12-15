/**
 * @file alg_power_limit.cpp
 * @author 乄乄乄 (850184312@qq.com)
 * @brief 功率限制
 * @version 1.1
 * @date 2024-03-21 0.1 24赛季定稿
 *
 * @copyright Copyright (c) 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_power_limit.h"
#include "dvc_djimotor.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 初始化参数
 *
 * @param __K1 参数K1的值，默认为1.23e-07f
 * @param __K2 参数K2的值，默认为1.453e-07f
 * @param __alpha 参数alpha的值，默认为4.081f
 * @param toque_coefficient 扭矩系数等的值，默认为1.99688994e-6f
 */
void Class_Power_Limit::Parameter_Init(float __K1 , float __K2 , float __alpha , float __toque_coefficient)
{
    k1 = __K1;
    k2 = __K2;
    Alpha = __alpha;
    Toque_Coefficient = __toque_coefficient;
}

/**
 * @brief 获取电机扭矩电流
 *
 * @param num 电机编号
 * @return float 电机扭矩电流
 */
float Class_Power_Limit::Get_Torque_Current(uint8_t num)
{
    return Torque_Current[num];
}

/**
 * @brief 设置底盘buffer
 *
 * @return float 电机扭矩电流
 */
void Class_Power_Limit::Set_Chassis_Buffer(float __buffer)
{
    Chassis_Buffer = __buffer;
}

/**
 * @brief 定时器周期到达回调函数
 *
 */
void Class_Power_Limit::TIM_Adjust_PeriodElapsedCallback()
{
	// //每个周期的总功率预测先清零
	// Total_Predict_Power = 0;
	// //根据总功率来计算比例系数K
	// for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
	// {
	// 	Predict_Power[i] =  Toque_Coefficient * Torque_Current[i] * Omega[i] * RAD_TO_RPM+
	// 						k2 * (Omega[i] * RAD_TO_RPM)* (Omega[i] * RAD_TO_RPM) +
	// 						k1 * Torque_Current[i] * Torque_Current[i] + 
    //                         Alpha;

	// 	if (Predict_Power[i] < 0) // negative power not included (transitory)
	// 		continue;
	// 	Total_Predict_Power += Predict_Power[i];
	// }

    // //开启功率限制
	// if (Total_Predict_Power > Total_Power_Limit) // determine if larger than max power
	// {
	// 	//计算伸缩系数k
	// 	Power_Scale = Total_Power_Limit / Total_Predict_Power;
	// 	for (uint8_t i = 0; i < 4; i++)
	// 	{
	// 		Scaled_Give_Power[i] = Predict_Power[i] * Power_Scale; // 获得各个电机伸缩后的功率限制
	// 		if (Scaled_Give_Power[i] < 0)
	// 		{
	// 			continue;
	// 		}

    //         //过程变量 equation_b equation_c
	// 		equation_b = Toque_Coefficient * Omega[i] * RAD_TO_RPM;
	// 		equation_c = k2 * (Omega[i] * RAD_TO_RPM ) * (Omega[i] * RAD_TO_RPM ) - Scaled_Give_Power[i] + Alpha;

	// 		if (Torque_Current[i] > 0) // Selection of the calculation formula according to the direction of the original moment
	// 		{
	// 			float temp = (-equation_b + sqrt(equation_b * equation_b - 4.0f * k1 * equation_c)) / (2.0f * k1);
	// 			if (temp > 16000)
	// 			{
	// 				Torque_Current[i] = 16000;
	// 			}
	// 			else
	// 				Torque_Current[i] = temp;
	// 				// Test_Current[i] = temp;
	// 		}
	// 		else
	// 		{
	// 			float temp = (-equation_b - sqrt(equation_b * equation_b - 4.0f * k1 * equation_c)) / (2.0f * k1);
	// 			if (temp < -16000)
	// 			{
	// 				Torque_Current[i] = -16000;
	// 			}
	// 			else
	// 				Torque_Current[i] = temp;
	// 				// Test_Current[i] = temp;
	// 		}
	// 	}
	// }
	
	Limit_K = (Chassis_Buffer-Min_Buffer)/60.0f;
	if(Limit_K<0) Limit_K = 0;
	for(int i=0;i<4;i++)
	{
		Torque_Current[i]*=Limit_K; 
	}
}

/**
 * @brief 设定缓冲区电流
 *
 */
void Class_Power_Limit::Output(Class_DJI_Motor_C620 (&Motor)[4])
{
    for(int i=0;i<4;i++)
	{
        Motor[i].CAN_Tx_Data[0] = (int16_t)Torque_Current[i] >> 8;
        Motor[i].CAN_Tx_Data[1] = (int16_t)Torque_Current[i];
    }
}

/**
 * @brief 设定四个电机的控制电流和当前角速度
 *
 */
void Class_Power_Limit::Set_Motor(Class_DJI_Motor_C620 (&Motor)[4])
{
    for(int i=0;i<4;i++)
    {
        Torque_Current[i] = Motor[i].Get_Out();
        Omega[i] = Motor[i].Get_Now_Omega();
    }
}

/* Function prototypes -------------------------------------------------------*/


/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
