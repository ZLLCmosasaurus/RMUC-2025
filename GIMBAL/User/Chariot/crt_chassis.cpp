/**
 * @file crt_chassis.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 底盘电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/**
 * @brief 轮组编号
 * 3 2
 *  1
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_chassis.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 底盘初始化
 *
 * @param __Chassis_Control_Type 底盘控制方式, 默认舵轮方式
 * @param __Speed 底盘速度限制最大值
 */
void Class_Tricycle_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max, float __Steer_Power_Ratio)
{
    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
    Steer_Power_Ratio = __Steer_Power_Ratio;


    //斜坡函数加减速速度X  0.005f, 0.01f
    Slope_Velocity_X.Init(0.25f,1.0f);
    //斜坡函数加减速速度Y  0.005f, 0.01f
    Slope_Velocity_Y.Init(0.25f,1.0f);
    //斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);

    //功率限制参数初始化
    Power_Limit.Parameter_Init();


    //电机PID批量初始化
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].PID_Omega.Init(1500.0f, 5000.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    }

    //轮向电机ID初始化
    Motor_Wheel[0].Init(&hcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hcan1, DJI_Motor_ID_0x204);

}


/**
 * @brief 速度解算
 *
 */
void Class_Tricycle_Chassis::Speed_Resolution(){
    //获取当前速度值，用于速度解算初始值获取
    switch (Chassis_Control_Type)
    {
        case (Chassis_Control_Type_DISABLE):
        {
            //底盘失能 四轮子自锁
            for (int i = 0; i < 4; i++)
            {
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Wheel[i].PID_Angle.Set_Integral_Error(0.0f);
                Motor_Wheel[i].Set_Target_Omega(0.0f);
            }            
        }
        break;
        case (Chassis_Control_Type_FLLOW || Chassis_Control_Type_SPIN):
        {
            //底盘四电机模式配置
            for (int i = 0; i < 4; i++)
            {
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            }
            //底盘限速
            if (Velocity_X_Max != 0)
            {
                Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
            }
            if (Velocity_Y_Max != 0)
            {
                Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
            }
            if (Omega_Max != 0)
            {
                Math_Constrain(&Omega_Max, -Omega_Max, Omega_Max);
            }

            //速度换算，正运动学分解
            float motor1_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
            float motor2_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
            float motor3_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
            float motor4_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);

            //线速度 cm/s  转角速度  RAD 
            float motor1_temp_rad = motor1_temp_linear_vel * VEL2RPM * RPM2RAD;
            float motor2_temp_rad = motor2_temp_linear_vel * VEL2RPM * RPM2RAD;
            float motor3_temp_rad = motor3_temp_linear_vel * VEL2RPM * RPM2RAD;
            float motor4_temp_rad = motor4_temp_linear_vel * VEL2RPM * RPM2RAD;
            //角速度*减速比  设定目标
            Motor_Wheel[0].Set_Target_Omega(  motor2_temp_rad * M3508_REDUCTION_RATIO);
            Motor_Wheel[1].Set_Target_Omega(- motor1_temp_rad * M3508_REDUCTION_RATIO);
            Motor_Wheel[2].Set_Target_Omega(- motor3_temp_rad * M3508_REDUCTION_RATIO);
            Motor_Wheel[3].Set_Target_Omega(  motor4_temp_rad * M3508_REDUCTION_RATIO);
              
            // max=find_max();
            // if(max>MAX_MOTOR_SPEED)
            // {
            //     Motor_Wheel[0].Set_Target_Omega(chassis_motor1.target_speed*MAX_MOTOR_SPEED*1.0/max);
            //     chassis_motor2.target_speed=(int)(chassis_motor2.target_speed*MAX_MOTOR_SPEED*1.0/max);
            //     chassis_motor3.target_speed=(int)(chassis_motor3.target_speed*MAX_MOTOR_SPEED*1.0/max);
            //     chassis_motor4.target_speed=(int)(chassis_motor4.target_speed*MAX_MOTOR_SPEED*1.0/max);
            // }
        }
        break;
    }   
}


/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback()
{
    //斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

    //速度解算
    Speed_Resolution();
    
    //各个电机具体PID
    for (int i = 0; i < 4; i++){
        Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
    }

   //功率限制
        // Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max());   //功率限制45w
//        Power_Limit.Set_Power_Limit(25.0f);
        Power_Limit.Set_Motor(Motor_Wheel);   //添加四个电机的控制电流和当前转速
        Power_Limit.Set_Chassis_Buffer(Referee->Get_Chassis_Energy_Buffer());
        Power_Limit.TIM_Adjust_PeriodElapsedCallback();  //功率限制算法
        Power_Limit.Output(Motor_Wheel);    //修改缓冲区

}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
