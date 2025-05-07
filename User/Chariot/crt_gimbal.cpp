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
float test_angle = 0;
float Test_Target_Omega = 0;
float last_angle = 0;
void Class_Gimbal_Yaw_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        // 默认开环速度控制
        Out = Out;
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        // 力矩环
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        // 角速度环
        PID_Omega.Set_Target(Target_Omega_Angle);
        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Yaw * 180.f / PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        // PID_Angle.Set_Target(Target_Angle);
        //  Target_Angle=test_angle;
        if (last_angle != Target_Angle)
        {
            PID_Angle.Set_Target(Target_Angle);
        }
        last_angle = Target_Angle;
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Yaw);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Radian = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Radian);
            PID_Omega.Set_Now(True_Gyro_Yaw * 57.3f);
        }
        else
        {
            // 角度环
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(-PID_Omega.Get_Out());
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

void Class_Gimbal_Yaw_Motor_GM6020::Disable()
{
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
    Set_Out(0.0f);
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
float test_omega = 1.0f;
float m_angle = 0.0f;
void Class_Gimbal_Pitch_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        // 默认开环
        Out = Out;
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        // 力矩环
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        // 角速度环

        //			if(True_Angle_Pitch>=15){
        //			Target_Omega_Angle=-test_omega;
        //			}
        //			if(True_Angle_Pitch<=-15){
        //				Target_Omega_Angle=test_omega;
        //			}

        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Pitch * 180.f / PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        // PID_Angle.Set_Target(-m_angle);
        PID_Angle.Set_Target(-Target_Angle);
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Pitch * 57.3);
        }
        else
        {
            // 角度环
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = -PID_Omega.Get_Out();
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

void Class_Gimbal_Pitch_Motor_GM6020::Disable()
{
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
    Set_Out(0.0f);
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_GM6020::Transform_Angle()
{
    True_Rad_Pitch = IMU->Get_Rad_Roll();
    True_Gyro_Pitch = IMU->Get_Gyro_Roll();
    True_Angle_Pitch = IMU->Get_Angle_Roll();
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
        // 角速度环
        PID_Omega.Set_Target(Target_Omega_Angle);
        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Pitch * 180.f / PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Out = PID_Omega.Get_Out();
        Set_Out(Out);
    }
    break;
    case (LK_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Pitch * 180.f / PI);
        }
        else
        {
            // 角度环
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
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
    // imu初始化
    Boardc_BMI.Init();

    // yaw轴电机
    Motor_Yaw.filtered_target_angle.Init(-30, 40, Filter_Fourier_Type_LOWPASS, 20, 0, 1000, 4);
    // 250 300
   Motor_Yaw.PID_Angle.Init(50.0f, 0.0f, 0.2f, 10.0f, 100, 1000, 0.0f, 0.0f, 0, 0.001f, 0.0f, PID_D_First_ENABLE);
    Motor_Yaw.PID_Omega.Init(300.0f, 2000.0f, 0.0f, 0.0f, 10000.0f, 20000.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, PID_D_First_ENABLE);
    Motor_Yaw.PID_Torque.Init(0.78f, 100.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.IMU = &Boardc_BMI;
    Motor_Yaw.Init(&hcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_IMU_ANGLE, 2048);

    // pitch轴电机
    Motor_Pitch.PID_Angle.Init(50.0f, 0.0f, 0.3f, 0.0f, 10000000, 10000000,0.0f, 0.0f, 0, 0.001f, 0.0f, PID_D_First_ENABLE);
    Motor_Pitch.PID_Omega.Init(100.0f, 2000.0f, 0.0f, 0, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max(), 0.0f, 0.0f, 0.0f, 0.001f, 0.8f);
    Motor_Pitch.PID_Torque.Init(0.8f, 100.0f, 0.0f, 0.0f, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());
    Motor_Pitch.IMU = &Boardc_BMI;
#ifdef DEBUG_PITCH_SPEED_LOOP
    Motor_Pitch.Init(&hcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_IMU_OMEGA, 3413);
#else
    Motor_Pitch.Init(&hcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_IMU_ANGLE, 3413);

#endif
}

/**
 * @brief 输出到电机
 *
 */
float temp_err = 0.0f;
float temp_target_angle = 0.0f;
void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        // 云台失能
        Motor_Pitch.Disable();
        Motor_Yaw.Disable();

        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Torque.Set_Integral_Error(0.0f);
    }
    else // 非失能模式
    {
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);

#ifdef DEBUG_PITCH_SPEED_LOOP
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_OMEGA);
#else
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);

#endif

        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
        }
        else if ((Gimbal_Control_Type == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            Target_Pitch_Angle = MiniPC->Get_Rx_Pitch_Angle();
            Target_Yaw_Angle = MiniPC->Get_Rx_Yaw_Angle();
        }

        // 限制角度范围 处理yaw轴180度问题
        while ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) > Max_Yaw_Angle)
        {
            Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
        }
        while ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) < -Max_Yaw_Angle)
        {
            Target_Yaw_Angle += (2 * Max_Yaw_Angle);
        }

        // 新处理yaw轴180度问题
        //  1. 角度优化

        //        float temp_min;

        //        // 计算误差，考虑当前电机状态
        //        temp_err = Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw();

        //        // 标准化到[0, 360)范围
        //        while (temp_err > 360.0f)
        //            temp_err -= 360.0f;
        //        while (temp_err < 0.0f)
        //            temp_err += 360.0f;

        //        // 比较路径长度
        //        if (fabs(temp_err) < (360.0f - fabs(temp_err)))
        //            temp_min = fabs(temp_err);
        //        else
        //            temp_min = 360.0f - fabs(temp_err);

        //        // 判断是否需要切换方向
        //        // if (temp_min > 90.0f)
        //        // {
        //        //     steering_wheel->invert_flag = !steering_wheel->invert_flag;
        //        //     // 重新计算误差
        //        //     temp_err = steering_wheel->Target_Angle - steering_wheel->Now_Angle - steering_wheel->invert_flag * 180.0f;
        //        // }
        //        // 2. 优劣弧优化，实际上角度优化那里已经完成了
        //        if (temp_err > 180.0f)
        //        {
        //            temp_err -= 360.0f;
        //        }
        //        else if (temp_err < -180.0f)
        //        {
        //            temp_err += 360.0f;
        //        }

        //        temp_target_angle = Motor_Yaw.Get_True_Angle_Yaw() + temp_err;
        //        Target_Yaw_Angle = temp_target_angle;

        // pitch限位
        Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

        // 设置目标角度
        Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
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

    // 根据不同c板的放置方式来修改这几个函数
    Motor_Yaw.Transform_Angle();
    Motor_Pitch.Transform_Angle();

    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
