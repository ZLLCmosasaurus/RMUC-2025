/**
 * @file alg_power_limit.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief power_limit
 * @version 0.1
 * @date 2023-08-29 0.1 23��������
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef ALG_POWER_LIMIT_H
#define ALG_POWER_LIMIT_H

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "arm_math.h"
#include "dvc_djimotor.h"
/* Exported macros -----------------------------------------------------------*/

#define RAD_TO_RPM  9.5493

class Class_DJI_Motor_C620;

class Class_Power_Limit
{
    public:

    void Parameter_Init(float __K1 = 1.23e-07f, float __K2 = 1.453e-07f, float __alpha = 4.081f, float __toque_coefficient = 1.99688994e-6f);

    inline void Set_Power_Limit(float __total_power_limit);

    float Get_Torque_Current(uint8_t num);

    void Set_Motor(Class_DJI_Motor_C620 (&Motor)[4]);
    void Set_Chassis_Buffer(float __buffer);
    void Output(Class_DJI_Motor_C620 (&Motor)[4]);

    void TIM_Adjust_PeriodElapsedCallback();
    
    protected:

    //转矩系数 rad转rpm系数
	float Toque_Coefficient = 1.99688994e-6f * (3591/187) / 13.93f;  // (20/16384)*(0.3)*(187/3591)/9.55

    //电机模型参数
	float k1 = 1.23e-07;		// k1
	float k2 = 1.453e-07;		// k2
	float Alpha = 4.081f;

    //过程变量
    float equation_b;
    float equation_c;

    //四电机目标力矩电流
    float Torque_Current[4];  
    //四电机当前角速度
    float Omega[4];	 
    //底盘总功率限制
    float Total_Power_Limit;  
    //底盘总预测功率
    float Total_Predict_Power = 0;  
    //预测功率
    float Predict_Power[4]; 
    //功率伸缩系数
	float Power_Scale;  
    //伸缩之后的功率限制
	float Scaled_Give_Power[4];  


    float Limit_K = 1.0f;
    float Chassis_Buffer;
    const float Min_Buffer = 10.0f; 
    //输出功率限制之后的电流到电机缓冲区
    void Output();

};

/**
 * @brief 设定总功率限制
 *
 */
void Class_Power_Limit::Set_Power_Limit(float __total_power_limit)
{
    Total_Power_Limit = __total_power_limit;
}

/* Exported types ------------------------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
