/**
 * @file crt_gimbal.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 云台电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

float Test_Pitch,Target_Pitch;
float Yaw_IMU_Angle,Pitch_IMU_Angle;
float Tmp_Target_Yaw_omega = 0.0f;
float Tmp_Ture_Yaw_omega = 0.0f;
float Test_Output_Yaw = 0.0f;
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float Test_Target_Omega=0;
void Class_Gimbal_Yaw_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环速度控制
        Set_Out(Target_Omega_Angle / Omega_Max * Output_Max);
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        //力矩环
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        //角速度环
        PID_Omega.Set_Target(Target_Omega_Angle);
        if (IMU->Get_IMU_Status()==IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Yaw*180.f/PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_IMU_Status()!=IMU_Status_DISABLE)
        {
            //角度环
            PID_Angle.Set_Now(True_Angle_Yaw);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            Tmp_Target_Yaw_omega = Target_Omega_Angle;//jscope曲线观测
            //速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Yaw*180.f/PI);

        }
        else
        {
            //角度环
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            //Target_Omega_Angle = PID_Angle.Get_Out();
            Target_Omega_Angle = 0.0f;

            //速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);

        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Test_Output_Yaw = PID_Omega.Get_Out();//jscope曲线观测
        
        Set_Out(-PID_Omega.Get_Out());//由于电机的输出值
    }
    break;
    case (DJI_Motor_Control_Method_ANGLE):
    {
        PID_Yaw_Encoder_Angle.Set_Target(Target_Angle);
        PID_Yaw_Encoder_Angle.Set_Now(Get_True_Angle_Yaw_From_Encoder());
        PID_Yaw_Encoder_Angle.TIM_Adjust_PeriodElapsedCallback();
        Target_Omega_Angle = PID_Yaw_Encoder_Angle.Get_Out();

        PID_Yaw_Encoder_Omega.Set_Target(Target_Omega_Angle);
        PID_Yaw_Encoder_Omega.Set_Now(-Data.Now_Omega_Angle);
        PID_Yaw_Encoder_Omega.TIM_Adjust_PeriodElapsedCallback();
        Out = -PID_Yaw_Encoder_Omega.Get_Out();

        Tmp_Target_Yaw_omega = Target_Omega_Angle;//jscope曲线观测
        Tmp_Ture_Yaw_omega = Data.Now_Omega_Angle;//jscope曲线观测
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Yaw_Motor_GM6020::Transform_Angle()
{
    True_Rad_Yaw = IMU->Get_Rad_Yaw();
    True_Gyro_Yaw = IMU->Get_Gyro_Yaw(); 
    True_Angle_Yaw = IMU->Get_Angle_Yaw();

    Tmp_Ture_Yaw_omega = True_Gyro_Yaw*180.f/PI;//jscope曲线观测
    Yaw_IMU_Angle = True_Angle_Yaw;//jscope曲线观测

}
void Class_Gimbal_Yaw_Motor_GM6020::Transform_EmcoderAngle_To_TrueAngle()
{
    // yaw轴total_angle
    float total_angle = ((float)Get_Now_Total_Encoder()) / 8191 * 2 * PI / 2;
    float Yaw_Now_Rad = fabsf(fmodf(total_angle, 2.0f * PI));
    float Yaw_Now_Angle = fabsf(Yaw_Now_Rad / PI * 180);
    if (Get_Now_Total_Round() < 0)
    {
        Yaw_Now_Rad = 2.0f * PI - Yaw_Now_Rad;
        Yaw_Now_Angle = fabsf(Yaw_Now_Rad / PI * 180);
    }
    if (Yaw_Now_Rad > PI)
    {
        Yaw_Now_Rad -= 2 * PI;
        Yaw_Now_Angle = Yaw_Now_Rad / PI * 180;
    }
    EmcoderAngle_To_TrueAngle = -Yaw_Now_Angle;
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Pitch_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环速度控制
        Set_Out(Target_Omega_Angle / Omega_Max * Output_Max);
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        //力矩环
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
		
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        //角速度环
        PID_Omega.Set_Target(Target_Omega_Angle);
        if (IMU->Get_IMU_Status()==IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Pitch*180.f/PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out());
				
    }
    break;
		
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);

        if (IMU->Get_IMU_Status()!=IMU_Status_DISABLE)
        {

            //角度环
            PID_Angle.Set_Now(True_Angle_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            //速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Pitch*180.f/PI);

        }
        else
        {
            //角度环
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            //速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);

        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(-PID_Omega.Get_Out() + Gravity_Compensate);
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_GM6020::Transform_Angle()
{
    True_Rad_Pitch = -1 * IMU->Get_Rad_Roll();
    True_Gyro_Pitch = -1 * IMU->Get_Gyro_Roll(); 
    True_Angle_Pitch = -1 * IMU->Get_Angle_Roll();  
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Pitch_Motor_LK6010::TIM_PID_PeriodElapsedCallback()
{
    switch (LK_Motor_Control_Method)
    {
    case (LK_Motor_Control_Method_TORQUE):
    {
        Out = Target_Torque*Torque_Current/Current_Max*Current_Max_Cmd;
        Set_Out(Out);
    }
    break;
    case (LK_Motor_Control_Method_IMU_OMEGA):
    {
        //角速度环
        PID_Omega.Set_Target(Target_Omega_Angle);
        if (IMU->Get_IMU_Status()==IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Pitch*180.f/PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Out = PID_Omega.Get_Out();
        Set_Out(Out);
    }
    break;
    case (LK_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_IMU_Status()!=IMU_Status_DISABLE)
        {
            //角度环
            PID_Angle.Set_Now(True_Angle_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            //速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Pitch*180.f/PI);
        }
        else
        {
            //角度环
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            //速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);

        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out() + Gravity_Compensate;
        Set_Out(Out);
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();   
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_LK6010::Transform_Angle()
{
    True_Rad_Pitch = 1 * IMU->Get_Rad_Pitch();
    True_Gyro_Pitch = 1 * IMU->Get_Gyro_Pitch(); 
    True_Angle_Pitch = 1 * IMU->Get_Angle_Pitch();
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Pitch_Motor_M2006::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环速度控制
        Set_Out(Target_Omega_Angle);
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        //角速度环
        PID_Omega.Set_Target(Target_Omega_Angle);
        if (IMU->Get_IMU_Status()==IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Pitch*180.f/PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out());
				
    }
    break;
		
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);

        if (IMU->Get_IMU_Status()!=IMU_Status_DISABLE)
        {

            //角度环
            PID_Angle.Set_Now(True_Angle_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            //速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Pitch*180.f/PI);

        }
        else
        {
            //角度环
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = 0.0f;

            //速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);

        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
    
        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out() + Gravity_Compensate);
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_M2006::Transform_Angle()
{
    True_Rad_Pitch = 1 * IMU->Get_Rad_Roll();
    True_Gyro_Pitch = 1 * IMU->Get_Gyro_Roll(); 
    True_Angle_Pitch = 1 * IMU->Get_Angle_Roll();  

    Target_Pitch = Target_Angle;
    Test_Pitch = True_Angle_Pitch;
	Pitch_IMU_Angle = True_Angle_Pitch;
}

#define YAW_IMU
/**
 * @brief 云台初始化
 *
 *///yaw电机初始化角度：47.21度
void Class_Gimbal::Init()
{
    #ifdef YAW_IMU
    //imu初始化
    Boardc_BMI.Init(); 
	//yaw轴电机 初始化
    //IMU初始化
    Motor_Yaw.PID_Angle.Init(80.0f, 0.016f, 0.04f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Yaw.PID_Omega.Init(150.0f, 0.15f, 0.0075f, 0.0f, 2000.0f, 20000.0f,0.0f, 0.0f, 0.0f, 0.001f);
    //编码器PID初始化
    Motor_Yaw.PID_Yaw_Encoder_Angle.Init(90.0f, 0.1f, 0.3f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.001f,0.005f);
    Motor_Yaw.PID_Yaw_Encoder_Omega.Init(85.0f, 0.1f, 0.0f, 0.0f, 7000.0f, 20000.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Yaw.IMU = &Boardc_BMI;
    Motor_Yaw.Init(&hcan2, DJI_Motor_ID_0x205,  DJI_Motor_Control_Method_IMU_ANGLE, 2);

    #endif
    //pitch轴电机
    Motor_Pitch.PID_Angle.Init(50.0f, 1.5f, 2.0f, 0.0f, 0.0f, 150.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Pitch.PID_Omega.Init(150.0f, 0.5f, 0.0f, 0.0f, 3000.0f, 10000.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Pitch.IMU = &Boardc_BMI;
    Motor_Pitch.Init(&hcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_IMU_ANGLE);
	

}


/**
 * @brief 输出到电机
 *
 */
float Tmp_Target_Yaw_Angle = 0.0f;
float Tmp_Ture_Yaw_Angle = 0.0f;
void Class_Gimbal::Output()
{

    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        //云台失能
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);

        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);

        Motor_Yaw.Set_Target_Omega_Angle(0.0f);
        Motor_Pitch.Set_Target_Omega_Angle(0.0f);
    }
    else // 非失能模式
    {   
        
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            //设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
        }
        else if((Gimbal_Control_Type == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status()!=MiniPC_Status_DISABLE))
        {   
            //Target_Pitch_Angle = MiniPC->Get_Rx_Pitch_Angle();
            Target_Yaw_Angle = MiniPC->Get_Rx_Yaw_Angle();          
        }

        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
        switch (Get_Launch_Mode())//吊射模式 拨杆左上 不影响自瞄
        {
        case Launch_Disable:
        {
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
            Tmp_Target_Yaw_Angle = Target_Yaw_Angle;
            Tmp_Ture_Yaw_Angle = Motor_Yaw.Get_True_Angle_Yaw();//IMU获取的真实角度
        }
        break;
        case Launch_Enable:
        {
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Tmp_Target_Yaw_Angle = Target_Yaw_Encoder_Angle;
            Tmp_Ture_Yaw_Angle = Motor_Yaw.Get_True_Angle_Yaw_From_Encoder();//编码器获取的真实角度
        }
        break;
        }

        //限制角度范围 处理yaw轴180度问题
        if((Tmp_Target_Yaw_Angle-Tmp_Ture_Yaw_Angle)>Max_Yaw_Angle)
        {
            Tmp_Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
        }
        else if((Tmp_Target_Yaw_Angle-Tmp_Ture_Yaw_Angle)<-Max_Yaw_Angle)
        {
            Tmp_Target_Yaw_Angle += (2 * Max_Yaw_Angle);
        }
				
        //pitch限位
        Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

        //设置目标角度
        Motor_Yaw.Set_Target_Angle(Tmp_Target_Yaw_Angle);
        Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle); 
    }
}
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    
	Output();
    
    //根据不同c板的放置方式来修改这几个函数
    Motor_Yaw.Transform_Angle();
    Motor_Yaw.Transform_EmcoderAngle_To_TrueAngle();
    Motor_Pitch.Transform_Angle();

	Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
	
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
