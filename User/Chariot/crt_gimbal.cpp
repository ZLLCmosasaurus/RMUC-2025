/**
 * @file crt_gimbal.cpp
 * @author lez by wanghongxi
 * @brief 云台
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/


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

            Target_Omega_Radian = PID_Angle.Get_Out();

            //速度环
            PID_Omega.Set_Target(Target_Omega_Radian);
            PID_Omega.Set_Now(True_Gyro_Yaw);

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
        Set_Out(PID_Omega.Get_Out());
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
            PID_Angle.Set_Now(True_Rad_Pitch);
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
void Class_Gimbal_Pitch_Motor_GM6020::Transform_Angle()
{
    True_Rad_Pitch = -1 * IMU->Get_Rad_Pitch();
    True_Gyro_Pitch = -1 * IMU->Get_Gyro_Pitch(); 
    True_Angle_Pitch = -1 * IMU->Get_Angle_Pitch();   
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
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    //imu初始化
    Boardc_BMI.Init(); 

    //yaw轴电机
    Motor_Yaw.PID_Angle.Init(0.65f, 0.0f, 0.0f, 0.0f, 3, 15,0.0f,0.0f,0,0.001f,0.0f);
    //250 300
    Motor_Yaw.PID_Omega.Init(6000.0f, 10.0f, 0.0f, 0.0f, 7000.0f, 20000.0f,0.0f,0.0f,0.0f,0.001f,0.0f);
    Motor_Yaw.PID_Torque.Init(0.78f, 100.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.IMU = &Boardc_BMI;
    Motor_Yaw.Init(&hcan2, DJI_Motor_ID_0x208, DJI_Motor_Control_Method_IMU_ANGLE, 2048);

    //pitch轴电机
    Motor_Pitch.PID_Angle.Init(1.2f, 0.0f, 0.1f, 0.0f, 6.0f * PI, 6.0f * PI);
    Motor_Pitch.PID_Omega.Init(400.0f, 50.0f, 0.0f, 0, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max(),0.0f,0.0f,0.0f,0.001f,0.8f);
    Motor_Pitch.PID_Torque.Init(0.8f, 100.0f, 0.0f, 0.0f, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());
    Motor_Pitch.IMU = &Boardc_BMI;
    Motor_Pitch.Init(&hcan1, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_IMU_ANGLE, 3413);

    //pitch轴电机 LK6010
    Motor_Pitch_LK6010.PID_Angle.Init(60.0f, 0.1f, 0.0f, 0.0f, 80.0f ,250.0f);
    //Motor_Pitch_LK6010.PID_Omega.Init(100.0f, 0.1f, 0.0f, 0, Motor_Pitch_LK6010.Get_Output_Max()/2.0f, Motor_Pitch_LK6010.Get_Output_Max(),0.0f,0.0f);
	Motor_Pitch_LK6010.PID_Omega.Init(1.0f, 0.0f, 0.0f, 0, Motor_Pitch_LK6010.Get_Output_Max()/2.0f, Motor_Pitch_LK6010.Get_Output_Max(),0.0f,0.0f,0.0f,0.001f,0.0f);
    Motor_Pitch_LK6010.PID_Torque.Init(0.8f, 100.0f, 0.0f, 0.0f, Motor_Pitch_LK6010.Get_Output_Max(), Motor_Pitch_LK6010.Get_Output_Max());
    Motor_Pitch_LK6010.IMU = &Boardc_BMI;
    Motor_Pitch_LK6010.Init(&hcan1, LK_Motor_ID_0x141, DJI_Motor_Control_Method_IMU_ANGLE);
}

/**
 * @brief 输出到电机
 *
 */
void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        //云台失能
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Pitch_LK6010.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_TORQUE);

        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch_LK6010.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch_LK6010.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch_LK6010.PID_Torque.Set_Integral_Error(0.0f);

        Motor_Yaw.Set_Target_Omega_Angle(0.0f);
        Motor_Pitch.Set_Target_Omega_Angle(0.0f);
        Motor_Pitch_LK6010.Set_Target_Omega_Angle(0.0f);
    }
    else // 非失能模式
    {   
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
        Motor_Pitch_LK6010.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);
        
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            //设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
            Motor_Pitch_LK6010.Set_Target_Angle(Target_Pitch_Angle);           
        }
        else if((Gimbal_Control_Type == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status()!=MiniPC_Status_DISABLE))
        {   
            Target_Pitch_Angle = MiniPC->Get_Rx_Pitch_Angle();
            Target_Yaw_Angle = MiniPC->Get_Rx_Yaw_Angle();          
        }

        //限制角度范围 处理yaw轴180度问题
        if((Target_Yaw_Angle-Motor_Yaw.Get_True_Angle_Yaw())>Max_Yaw_Angle)
        {
            Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
        }
        else if((Target_Yaw_Angle-Motor_Yaw.Get_True_Angle_Yaw())<-Max_Yaw_Angle)
        {
            Target_Yaw_Angle += (2 * Max_Yaw_Angle);
        }
    
        //pitch限位
        Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

        //设置目标角度
        Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
        Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
        Motor_Pitch_LK6010.Set_Target_Angle(Target_Pitch_Angle); 
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
    Motor_Pitch.Transform_Angle();
    Motor_Pitch_LK6010.Transform_Angle();


    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch_LK6010.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
