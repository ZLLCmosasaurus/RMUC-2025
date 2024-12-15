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

float __Target_Omega;
float test = -10;
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Yaw_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        // 默认开环速度控制
        Set_Out(Target_Omega / Omega_Max * Output_Max);
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        // if (True_Angle_Yaw > 70)
        //     Target_Omega = test;
        // if (True_Angle_Yaw < 20)
        //     Target_Omega = -test;
        PID_Omega.Set_Target(Target_Omega);
        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega);

            // PID_Omega.Set_K_P(1500.0f);
            // PID_Omega.Set_K_I(3000.0f);
            // PID_Omega.Set_K_D(0.15f);
            // PID_Torque.Set_K_P(2.0f);
            // PID_Torque.Set_K_I(15.0f);
            // PID_Torque.Set_K_D(0.0f);
        }
        else
        {
            PID_Angle.Set_Now(True_Angle_Yaw);
            PID_Omega.Set_Now(RADPS_TO_RPM(True_Gyro_Yaw));
            // PID_Omega.Set_K_P(10000.0f);
            // PID_Omega.Set_K_I(100000.0f);
            // PID_Omega.Set_K_D(0.1f);
            // PID_Torque.Set_K_P(0.78f);
            // PID_Torque.Set_K_I(100.0f);
            // PID_Torque.Set_K_D(0.0f);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        // PID_Torque.Set_Target(Target_Torque);
        // PID_Torque.Set_Now(Data.Now_Torque);
        // PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        // Set_Out(PID_Torque.Get_Out());
        Set_Out(PID_Omega.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);

            // PID_Angle.Set_K_P(7.0f);
            // PID_Angle.Set_K_I(0.0f);
            // PID_Angle.Set_K_D(0.1f);
            // PID_Omega.Set_K_P(1500.0f);
            // PID_Omega.Set_K_I(3000.0f);
            // PID_Omega.Set_K_D(0.15f);
            // PID_Torque.Set_K_P(2.0f);
            // PID_Torque.Set_K_I(15.0f);
            // PID_Torque.Set_K_D(0.0f);
        }
        else
        {
            PID_Angle.Set_Now(True_Angle_Yaw);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(RADPS_TO_RPM(True_Gyro_Yaw));
            // PID_Angle.Set_K_P(28.0f);
            // PID_Angle.Set_K_I(0.0f);
            // PID_Angle.Set_K_D(0.03f);
            // PID_Omega.Set_K_P(10000.0f);
            // PID_Omega.Set_K_I(100000.0f);
            // PID_Omega.Set_K_D(0.1f);
            // PID_Torque.Set_K_P(0.78f);
            // PID_Torque.Set_K_I(100.0f);
            // PID_Torque.Set_K_D(0.0f);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        // PID_Torque.Set_Target(Target_Torque);
        // PID_Torque.Set_Now(Data.Now_Torque);
        // PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        // Set_Out(PID_Torque.Get_Out());
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
    // if (IMU->Get_Angle_Yaw() > 0)
    //     True_Angle_Yaw = IMU->Get_Angle_Yaw() - 180;
    // if (IMU->Get_Angle_Yaw() < 0)
    //     True_Angle_Yaw = IMU->Get_Angle_Yaw() + 180;
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
        // 默认开环速度控制
        Set_Out(Target_Omega / Omega_Max * Output_Max);
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        // if (RAD_TO_ANGEL(True_Rad_Pitch) > 20)
        //     Target_Omega = test;
        // if (RAD_TO_ANGEL(True_Rad_Pitch) < -20)
        //     Target_Omega = -test;
        PID_Omega.Set_Target(Target_Omega);
        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega);

            // PID_Omega.Set_K_P(1000.0f);
            // PID_Omega.Set_K_I(5000.0f);
            // PID_Omega.Set_K_D(0.7f);
            // PID_Torque.Set_K_P(2.0f);
            // PID_Torque.Set_K_I(15.0f);
            // PID_Torque.Set_K_D(0.0f);
        }
        else
        {

            PID_Angle.Set_Now(RAD_TO_ANGEL(True_Rad_Pitch));
            PID_Omega.Set_Now(RADPS_TO_RPM(True_Gyro_Pitch));

            // PID_Omega.Set_K_P(8000.0f);
            // PID_Omega.Set_K_I(100000.0f);
            // PID_Omega.Set_K_D(0.0003f);
            // PID_Torque.Set_K_P(0.8f);
            // PID_Torque.Set_K_I(100.0f);
            // PID_Torque.Set_K_D(0.0f);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = -PID_Omega.Get_Out();

        // PID_Torque.Set_Target(Target_Torque);
        // PID_Torque.Set_Now(Data.Now_Torque);
        // PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(Target_Torque);
    }
    break;
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);

        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega = -PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);

            // PID_Angle.Set_K_P(7.0f);
            // PID_Angle.Set_K_I(0.0f);
            // PID_Angle.Set_K_D(0.1f);
            // PID_Omega.Set_K_P(1000.0f);
            // PID_Omega.Set_K_I(5000.0f);
            // PID_Omega.Set_K_D(0.7f);
            // PID_Torque.Set_K_P(2.0f);
            // PID_Torque.Set_K_I(15.0f);
            // PID_Torque.Set_K_D(0.0f);
        }
        else
        {
            PID_Angle.Set_Now(RAD_TO_ANGEL(True_Rad_Pitch));
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(RADPS_TO_RPM(True_Gyro_Pitch));
            // PID_Angle.Set_K_P(16.5f);
            // PID_Angle.Set_K_I(0.0f);
            // PID_Angle.Set_K_D(0.01f);
            // PID_Omega.Set_K_P(8000.0f);
            // PID_Omega.Set_K_I(100000.0f);
            // PID_Omega.Set_K_D(0.0003f);
            // PID_Torque.Set_K_P(0.8f);
            // PID_Torque.Set_K_I(100.0f);
            // PID_Torque.Set_K_D(0.0f);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = -PID_Omega.Get_Out();

        // PID_Torque.Set_Target(Target_Torque);
        // PID_Torque.Set_Now(Data.Now_Torque);
        // PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        // Set_Out(PID_Torque.Get_Out() + Gravity_Compensate);
        Set_Out(Target_Torque + Gravity_Compensate);
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
    True_Rad_Pitch =  -IMU->Get_Rad_Roll();
    True_Gyro_Pitch =  -IMU->Get_Gyro_Pitch();
    True_Angle_Pitch = RAD_TO_ANGEL(True_Rad_Pitch);
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
        Out = Target_Torque * Torque_Current / Current_Max * Current_Max_Cmd;
        Set_Out(Out);
    }
    break;
    case (LK_Motor_Control_Method_IMU_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega);
        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega);

            // PID_Omega.Set_K_P(1000.0f);
            // PID_Omega.Set_K_I(5000.0f);
            // PID_Omega.Set_K_D(0.7f);
            // PID_Torque.Set_K_P(2.0f);
            // PID_Torque.Set_K_I(15.0f);
            // PID_Torque.Set_K_D(0.0f);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Pitch);

            // PID_Omega.Set_K_P(8000.0f);
            // PID_Omega.Set_K_I(100000.0f);
            // PID_Omega.Set_K_D(0.0003f);
            // PID_Torque.Set_K_P(0.8f);
            // PID_Torque.Set_K_I(100.0f);
            // PID_Torque.Set_K_D(0.0f);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Out = PID_Omega.Get_Out();
        Set_Out(Out);
    }
    break;
    case (LK_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);

        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();
            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);

            // PID_Angle.Set_K_P(7.0f);
            // PID_Angle.Set_K_I(0.0f);
            // PID_Angle.Set_K_D(0.1f);
            // PID_Omega.Set_K_P(1000.0f);
            // PID_Omega.Set_K_I(5000.0f);
            // PID_Omega.Set_K_D(0.7f);
            // PID_Torque.Set_K_P(2.0f);
            // PID_Torque.Set_K_I(15.0f);
            // PID_Torque.Set_K_D(0.0f);
        }
        else
        {
            PID_Angle.Set_Now(True_Rad_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();
            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(True_Gyro_Pitch);

            // PID_Angle.Set_K_P(16.5f);
            // PID_Angle.Set_K_I(0.0f);
            // PID_Angle.Set_K_D(0.01f);
            // PID_Omega.Set_K_P(8000.0f);
            // PID_Omega.Set_K_I(100000.0f);
            // PID_Omega.Set_K_D(0.0003f);
            // PID_Torque.Set_K_P(0.8f);
            // PID_Torque.Set_K_I(100.0f);
            // PID_Torque.Set_K_D(0.0f);
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
    True_Rad_Pitch = -1 * IMU->Get_Rad_Pitch();
    True_Gyro_Pitch = -1 * IMU->Get_Gyro_Pitch();
    True_Angle_Pitch = -1 * IMU->Get_Angle_Pitch();
}

/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();

    // yaw轴电机
    // Motor_Yaw.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 100, 100);
    // Motor_Yaw.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());

    Motor_Yaw.PID_Angle.Init(1.2f, 0.0f, 0.0f, 0.0f, 500, 500, 0, 0, 0, 0.001, 0.15);
    Motor_Yaw.PID_Omega.Init(3500.0f, 10000.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());

    // Motor_Yaw.PID_Angle.Init(9.35f, 0.0f, 0.071f, 0.0f, 100, 100);
    // Motor_Yaw.PID_Omega.Init(150.0f, 15000.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());

    Motor_Yaw.PID_Torque.Init(0.78f, 100.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.IMU = &Boardc_BMI;
    Motor_Yaw.Init(&hcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_IMU_ANGLE, 2048);

    // pitch轴电机
    //  Motor_Pitch.PID_Angle.Init(9.35f, 0.0f, 0.071f, 0.0f, 100, 100);
    //  Motor_Pitch.PID_Omega.Init(150.0f, 15000.0f, 0.0f, 0.0f, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());

    // Motor_Pitch.PID_Angle.Init(7.0f, 2.0f, 0.0f, 0.0f, 100, 100);
    // Motor_Pitch.PID_Omega.Init(2200.0f, 15000.0f, 0.0f, 0.0f, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());

    Motor_Pitch.PID_Angle.Init(5.5f, 0.0f, 0.007f, 0.0f, 100, 100, 0, 0, 0, 0.001, 0.005);
    Motor_Pitch.PID_Omega.Init(2200.0f, 3500.0f, 0.0f, 0.0f, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());

    Motor_Pitch.PID_Torque.Init(0.8f, 100.0f, 0.0f, 0.0f, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());
    Motor_Pitch.IMU = &Boardc_BMI;
    Motor_Pitch.Init(&hcan2, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_IMU_ANGLE, 3413);

    // pitch轴电机 LK6010
    Motor_Pitch_LK6010.PID_Angle.Init(10.0f, 0.0f, 0.1f, 0.0f, 6.0f * PI, 6.0f * PI);
    Motor_Pitch_LK6010.PID_Omega.Init(65.0f, 0.5f, 0.0f, 0, Motor_Pitch_LK6010.Get_Output_Max(), Motor_Pitch_LK6010.Get_Output_Max(), 0.0f, 0.0f);
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
        // 云台失能
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

        Motor_Yaw.Set_Target_Omega(0.0f);
        Motor_Pitch.Set_Target_Omega(0.0f);
        Motor_Pitch_LK6010.Set_Target_Omega(0.0f);
    }
    else
    {
        // 云台工作
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
        Motor_Pitch_LK6010.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);

        // if (Target_Yaw_Angle>180)
        //     Target_Yaw_Angle = -180;
        // if (Target_Yaw_Angle < -180)
        //     Target_Yaw_Angle = 180;
        // Math_Constrain(&Target_Yaw_Angle, -Max_Yaw_Angle, Max_Yaw_Angle);
        // pitch限位
        Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

        // if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        // {
        //     // 限制角度范围 处理yaw轴180度问题
        //     if ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) > Max_Yaw_Angle)
        //     {
        //         Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
        //     }
        //     else if ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) < -Max_Yaw_Angle)
        //     {
        //         Target_Yaw_Angle += (2 * Max_Yaw_Angle);
        //     }
        //     // 设置目标角度
        //     Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
        //     Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
        //     Motor_Pitch_LK6010.Set_Target_Angle(Target_Pitch_Angle);
        // }
        // else if ((Gimbal_Control_Type == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        // {
        //     // 设置目标角度
        //     Motor_Yaw.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle());

        //     if ((Motor_Yaw.Get_Target_Angle() - Motor_Yaw.Get_True_Angle_Yaw()) > 180)
        //     {
        //         Motor_Yaw.Set_Target_Angle(Motor_Yaw.Get_Target_Angle() - 360);
        //     }
        //     if ((Motor_Yaw.Get_Target_Angle() - Motor_Yaw.Get_True_Angle_Yaw()) < -180)
        //     {
        //         Motor_Yaw.Set_Target_Angle(Motor_Yaw.Get_Target_Angle() + 360);
        //     }

        //     Motor_Pitch.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle());
        //     Motor_Pitch_LK6010.Set_Target_Angle(Target_Pitch_Angle);
        // }

        // 限制角度范围 处理yaw轴180度问题
        if ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) > Max_Yaw_Angle)
        {
            Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
        }
        else if ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) < -Max_Yaw_Angle)
        {
            Target_Yaw_Angle += (2 * Max_Yaw_Angle);
        }
        // 设置目标角度
        Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
        Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
        Motor_Pitch_LK6010.Set_Target_Angle(Target_Pitch_Angle);
    }
}

/**
 * @brief 计算云台yaw总角度
 *
 */
void Class_Gimbal::Calculate_Total_Angle()
{
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{

    Output();

    // 根据不同c板的放置方式来修改这几个函数
    Motor_Yaw.Transform_Angle();
    Motor_Pitch.Transform_Angle();
    Motor_Pitch_LK6010.Transform_Angle();

    Motor_Yaw.TIM_PID_PeriodElapsedCallback();

    static int mod200 = 0;
    mod200++;
    if (mod200 == 5)
    {

        mod200 = 0;
    }

    Motor_Pitch.TIM_PID_PeriodElapsedCallback();

    Motor_Pitch_LK6010.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
