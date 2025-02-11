/**
 * @file crt_gimbal.cpp
 * @author cjw
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
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();

    // yaw轴电机
    Motor_Yaw_A.PID_Angle.Init(20.f, 0.0f, 0.0f, 0.0f, Motor_Yaw_A.Get_Output_Max(), Motor_Yaw_A.Get_Output_Max());
    Motor_Yaw_A.PID_Omega.Init(20.0f, 40.0f, 0.0f, 0.0f, 4000, Motor_Yaw_A.Get_Output_Max(), 50, 100);
    Motor_Yaw_A.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Yaw_A.Get_Output_Max(), Motor_Yaw_A.Get_Output_Max());
    Motor_Yaw_A.Init(&hfdcan2, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 2048);

    Motor_Yaw_B.PID_Angle.Init(40.f, 0.0f, 0.0f, 0.0f, Motor_Yaw_B.Get_Output_Max(), Motor_Yaw_B.Get_Output_Max());
    Motor_Yaw_B.PID_Omega.Init(25.0f, 200.0f, 0.0f, 0.0f, 8000, Motor_Yaw_B.Get_Output_Max(), 50, 100);
    Motor_Yaw_B.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Yaw_B.Get_Output_Max(), Motor_Yaw_B.Get_Output_Max());
    Motor_Yaw_B.Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 2048);

    Motor_Main_Yaw.PID_Angle.Init(0.15f, 0.0f, 0.0f, 0.0f, 3, 15, 0.0f, 0.0f, 0, 0.001f, 0.0f);
    Motor_Main_Yaw.PID_Omega.Init(1000.0f, 5.0f, 0.0f, 0.0f, 100.0f, 2048.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f);
    Motor_Main_Yaw.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Main_Yaw.Get_Output_Max(), Motor_Main_Yaw.Get_Output_Max());
    Motor_Main_Yaw.Init(&hfdcan3, LK_Motor_ID_0x141, LK_Motor_Control_Method_ANGLE, 2048);

    // pitch轴电机
    Motor_Pitch_A.PID_Angle.Init(1.5f, 0.0f, 0.0f, 0.0f, 20, 50);
    Motor_Pitch_A.PID_Omega.Init(400.0f, 100.0f, 0.0f, 0.0f, 8000, Motor_Pitch_A.Get_Output_Max(), 5, 10);
    Motor_Pitch_A.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Pitch_A.Get_Output_Max(), Motor_Pitch_A.Get_Output_Max());
    Motor_Pitch_A.Init(&hfdcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE, 3413);

    Motor_Pitch_B.PID_Angle.Init(40.f, 0.0f, 0.0f, 0.0f, Motor_Pitch_B.Get_Output_Max(), Motor_Pitch_B.Get_Output_Max());
    Motor_Pitch_B.PID_Omega.Init(25.0f, 200.0f, 0.0f, 0.0f, 8000, Motor_Pitch_B.Get_Output_Max(), 50, 100);
    Motor_Pitch_B.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Pitch_B.Get_Output_Max(), Motor_Pitch_B.Get_Output_Max());
    Motor_Pitch_B.Init(&hfdcan1, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE, 3413);

    Motor_Main_Yaw.Set_Zero_Position(31.2346f);
    Motor_Yaw_A.Set_Zero_Position(294.9169f);
    Motor_Yaw_B.Set_Zero_Position(88.9013f);
    Motor_Pitch_A.Set_Zero_Position(47.4169f);
    Motor_Pitch_B.Set_Zero_Position(21.8408f);
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
        Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_TORQUE);
        Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);

        Motor_Yaw_A.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw_A.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw_A.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Yaw_B.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw_B.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw_B.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch_A.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch_A.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch_A.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch_B.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch_B.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch_B.PID_Torque.Set_Integral_Error(0.0f);

        Motor_Yaw_A.Set_Target_Torque(0.0f);
        Motor_Yaw_B.Set_Target_Torque(0.0f);
        Motor_Main_Yaw.Set_Target_Torque(0.0f);
        Motor_Pitch_A.Set_Target_Torque(0.0f);
        Motor_Pitch_B.Set_Target_Torque(0.0f);

        Motor_Main_Yaw.Set_Out(0.0f);
        Motor_Yaw_A.Set_Out(0.0f);
        Motor_Yaw_B.Set_Out(0.0f);
        Motor_Pitch_A.Set_Out(0.0f);
        Motor_Pitch_B.Set_Out(0.0f);
    }
    else // 非失能模式
    {
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
            Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            Math_Constrain(&Target_Pitch_Angle_A, Min_Pitch_Angle, Max_Pitch_Angle);
            Math_Constrain(&Target_Pitch_Angle_B, Min_Pitch_Angle, Max_Pitch_Angle);
            Math_Constrain(&Target_Yaw_Angle_A, Min_Yaw_Angle_A, Max_Yaw_Angle_A);
            Math_Constrain(&Target_Yaw_Angle_B, Min_Yaw_Angle_B, Max_Yaw_Angle_B);

            // 设置目标角度
            Motor_Yaw_A.Set_Target_Angle(0.0f);
            Motor_Yaw_B.Set_Target_Angle(0.0f);
            Motor_Main_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch_A.Set_Target_Angle(Target_Pitch_Angle_A);
            Motor_Pitch_B.Set_Target_Angle(Target_Pitch_Angle_A);
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            // 大yaw不动
            Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
            Motor_Main_Yaw.Set_Target_Angle(Boardc_BMI.Get_Angle_Yaw());

            // A云台控制逻辑
            if (MiniPC->Get_Auto_aim_Status_A() == Auto_aim_Status_DISABLE)
            {
                // 云台控制方式
                Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                if(A_Cruise_Flag == 0)
                {
                     Motor_Yaw_A.Set_Target_Omega_Angle(CRUISE_SPEED_YAW);
                     A_Cruise_Flag = 1;
                }               
                if (Get_True_Angle_Yaw_A() <= -170.f)
                    Motor_Yaw_A.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW);
                else if (Get_True_Angle_Yaw_A() <= -10.f && Get_True_Angle_Yaw_A() >= -20.0f)
                    Motor_Yaw_A.Set_Target_Omega_Angle(CRUISE_SPEED_YAW);

                if(Motor_Pitch_A.Get_Target_Omega_Angle() == 0.f)
                    Motor_Pitch_A.Set_Target_Omega_Angle(0.5f);

                if (Get_True_Angle_Pitch_A() >= 12.0f)
                    Motor_Pitch_A.Set_Target_Omega_Angle(-0.5f);
                else if (Get_True_Angle_Pitch_A() <= 0.0f)
                    Motor_Pitch_A.Set_Target_Omega_Angle(0.5f);
            }
            else if (MiniPC->Get_Auto_aim_Status_A() == Auto_aim_Status_ENABLE)
            {
                Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                Motor_Yaw_A.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle_A());
                Motor_Pitch_A.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_A());

                A_Cruise_Flag = 0;
            }
            // B云台控制逻辑
            if (MiniPC->Get_Auto_aim_Status_B() == Auto_aim_Status_DISABLE)
            {
                // 云台控制方式
                Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

                if(B_Cruise_Flag == 0)
                {
                    Motor_Yaw_B.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW);
                    B_Cruise_Flag = 1;
                }

                if (Get_True_Angle_Yaw_B() >= 175.f)
                {
                     Motor_Yaw_B.Set_Target_Omega_Angle(CRUISE_SPEED_YAW);
                }
                   
                else if (Get_True_Angle_Yaw_B() >= 10.f && Get_True_Angle_Yaw_B() <= 20.0f)
                {
                    Motor_Yaw_B.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW);
                }
                    
                if(Motor_Pitch_B.Get_Target_Omega_Angle() == 0.f)
                    Motor_Pitch_B.Set_Target_Omega_Angle(CRUISE_SPEED_PITCH);

                if (Get_True_Angle_Pitch_B() >= 12.0f)
                    Motor_Pitch_B.Set_Target_Omega_Angle(-CRUISE_SPEED_PITCH);
                else if (Get_True_Angle_Pitch_B() <= 0.0f)
                    Motor_Pitch_B.Set_Target_Omega_Angle(CRUISE_SPEED_PITCH);
            }
            else if (MiniPC->Get_Auto_aim_Status_B() == Auto_aim_Status_ENABLE)
            {
                Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                Motor_Yaw_B.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle_B());
                Motor_Pitch_B.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_B());

                B_Cruise_Flag = 0;
            }
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
        {

            Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
            Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            Motor_Main_Yaw.Set_Target_Angle(Boardc_BMI.Get_Angle_Yaw());
            Motor_Pitch_A.Set_Target_Angle(Get_True_Angle_Pitch_A());
            Motor_Pitch_B.Set_Target_Angle(Get_True_Angle_Pitch_B());
            Motor_Yaw_A.Set_Target_Angle(Get_True_Angle_Yaw_A());
            Motor_Yaw_B.Set_Target_Angle(Get_True_Angle_Yaw_B());
        }
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    Output();

    // 根据零位的不同来修改这几个函数
    Yaw_Angle_Transform_A();
    Yaw_Angle_Transform_B();
    Pitch_Angle_Transform_A();
    Pitch_Angle_Transform_B();
    Yaw_Angle_Transform_Main();

    MiniPC_Update(); // 上位机数据更新
    Yaw_Angle_Limit(Yaw_A);
    Yaw_Angle_Limit(Yaw_B);

    Motor_Yaw_A.TIM_PID_PeriodElapsedCallback();
    Motor_Yaw_B.TIM_PID_PeriodElapsedCallback();
    Motor_Main_Yaw.TIM_Process_PeriodElapsedCallback();
    Motor_Pitch_A.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch_B.TIM_PID_PeriodElapsedCallback();

    PID_Update();
}

void Class_Gimbal::Yaw_Angle_Transform_Main()
{
    // LK电机正方向未设置 不一定有效 6020有效
    float temp_yaw;
    if (Motor_Main_Yaw.Get_Now_Angle() > Motor_Main_Yaw.Get_Zero_Position())
    {
        temp_yaw = (-(Motor_Main_Yaw.Get_Now_Angle() - Motor_Main_Yaw.Get_Zero_Position())); // 电机数据转现实坐标系
        if (temp_yaw < -180.0f)
            temp_yaw += 360.0f;
    }
    else if (Motor_Main_Yaw.Get_Now_Angle() < Motor_Main_Yaw.Get_Zero_Position())
    {
        temp_yaw = (Motor_Main_Yaw.Get_Zero_Position() - Motor_Main_Yaw.Get_Now_Angle());
        if (temp_yaw > 180.0f)
            temp_yaw -= 360.0f;
    }
    Set_True_Angle_Yaw_Main(-temp_yaw);
    Control_Update_Main();
}
void Class_Gimbal::Yaw_Angle_Transform_A()
{
    float temp_yaw;
    if (Motor_Yaw_A.Get_Now_Angle() > Motor_Yaw_A.Get_Zero_Position())
    {
        temp_yaw = (-(Motor_Yaw_A.Get_Now_Angle() - Motor_Yaw_A.Get_Zero_Position())); // 电机数据转现实坐标系
        if (temp_yaw < -180.0f)
            temp_yaw += 360.0f;
    }
    else if (Motor_Yaw_A.Get_Now_Angle() < Motor_Yaw_A.Get_Zero_Position())
    {
        temp_yaw = (Motor_Yaw_A.Get_Zero_Position() - Motor_Yaw_A.Get_Now_Angle());
        if (temp_yaw > 180.0f)
            temp_yaw -= 360.0f;
    }
    Set_True_Angle_Yaw_A(-temp_yaw);

    // 边缘解算
    int invert_flag = 0;
    float temp_error = 0, temp_min = 0, pre_angle = 0, pre_omega = 0;

    temp_error = Get_Target_Yaw_Angle_A() - Get_True_Angle_Yaw_A() - invert_flag * 180;
    while (temp_error > 360.0f)
        temp_error -= 360.0f;
    while (temp_error < 0.0f)
        temp_error += 360.0f;
    if (fabs(temp_error) < (360.0f - fabs(temp_error)))
        temp_min = fabs(temp_error);
    else
        temp_min = 360.0f - fabs(temp_error);
    if (temp_min > 180.0f)
    {
        invert_flag = !invert_flag;
        // 重新计算误差
        temp_error = Get_Target_Yaw_Angle_A() - Get_True_Angle_Yaw_A() - invert_flag * 180.0f;
    }

    if (temp_error > 180.0f)
        temp_error -= 360.0f;
    else if (temp_error < -180.0f)
        temp_error += 360.0f;

    Set_Target_Yaw_Angle_A(Get_True_Angle_Yaw_A() + temp_error);
    Motor_Yaw_A.Set_Transform_Angle(Get_True_Angle_Yaw_A());
    Motor_Yaw_A.Set_Transform_Omega(Motor_Yaw_A.Get_Now_Omega_Angle());
}
void Class_Gimbal::Yaw_Angle_Transform_B()
{
    float temp_yaw;
    // 如果电机Yaw_B的当前角度大于零位置
    if (Motor_Yaw_B.Get_Now_Angle() > Motor_Yaw_B.Get_Zero_Position())
    {
        temp_yaw = (-(Motor_Yaw_B.Get_Now_Angle() - Motor_Yaw_B.Get_Zero_Position())); // 电机数据转现实坐标系
        if (temp_yaw < -180.0f)
            temp_yaw += 360.0f;
    }
    else if (Motor_Yaw_B.Get_Now_Angle() < Motor_Yaw_B.Get_Zero_Position())
    {
        temp_yaw = (Motor_Yaw_B.Get_Zero_Position() - Motor_Yaw_B.Get_Now_Angle());
        if (temp_yaw > 180.0f)
            temp_yaw -= 360.0f;
    }
    Set_True_Angle_Yaw_B(-temp_yaw);
    Motor_Yaw_B.Set_Transform_Angle(Get_True_Angle_Yaw_B());
    Motor_Yaw_B.Set_Transform_Omega(Motor_Yaw_B.Get_Now_Omega_Angle());
    Motor_Yaw_B.Set_Transform_Torque(-Motor_Yaw_B.Get_Now_Torque());

    // 边缘解算
    int invert_flag = 0;
    float temp_error = 0, temp_min = 0, pre_angle = 0, pre_omega = 0;

    temp_error = Get_Target_Yaw_Angle_B() - Get_True_Angle_Yaw_B() - invert_flag * 180;
    while (temp_error > 360.0f)
        temp_error -= 360.0f;
    while (temp_error < 0.0f)
        temp_error += 360.0f;
    if (fabs(temp_error) < (360.0f - fabs(temp_error)))
        temp_min = fabs(temp_error);
    else
        temp_min = 360.0f - fabs(temp_error);
    if (temp_min > 180.0f)
    {
        invert_flag = !invert_flag;
        // 重新计算误差
        temp_error = Get_Target_Yaw_Angle_B() - Get_True_Angle_Yaw_B() - invert_flag * 180.0f;
    }

    if (temp_error > 180.0f)
        temp_error -= 360.0f;
    else if (temp_error < -180.0f)
        temp_error += 360.0f;

    Set_Target_Yaw_Angle_B(Get_True_Angle_Yaw_B() + temp_error);
    Motor_Yaw_B.Set_Transform_Angle(Get_True_Angle_Yaw_B());
    Motor_Yaw_B.Set_Transform_Omega(Motor_Yaw_B.Get_Now_Omega_Angle());
}

void Class_Gimbal::Pitch_Angle_Transform_A()
{
    float temp_pitch;
    // 如果电机Pitch_A的当前角度大于零位置
    if (Motor_Pitch_A.Get_Now_Angle() > Motor_Pitch_A.Get_Zero_Position())
    {
        temp_pitch = (-(Motor_Pitch_A.Get_Now_Angle() - Motor_Pitch_A.Get_Zero_Position())); // 电机数据转现实坐标系
        if (temp_pitch < -180.0f)
            temp_pitch += 360.0f;
    }
    else if (Motor_Pitch_A.Get_Now_Angle() < Motor_Pitch_A.Get_Zero_Position())
    {
        temp_pitch = (Motor_Pitch_A.Get_Zero_Position() - Motor_Pitch_A.Get_Now_Angle());
        if (temp_pitch > 180.0f)
            temp_pitch -= 360.0f;
    }
    Set_True_Angle_Pitch_A(temp_pitch);
    Motor_Pitch_A.Set_Transform_Angle(Get_True_Angle_Pitch_A());
    Motor_Pitch_A.Set_Transform_Omega(-Motor_Pitch_A.Get_Now_Omega_Radian());
    Motor_Pitch_A.Set_Transform_Torque(Motor_Pitch_A.Get_Now_Torque());
}

void Class_Gimbal::Pitch_Angle_Transform_B()
{
    float temp_pitch;
    if (Motor_Pitch_B.Get_Now_Angle() > Motor_Pitch_B.Get_Zero_Position())
    {
        temp_pitch = (-(Motor_Pitch_B.Get_Now_Angle() - Motor_Pitch_B.Get_Zero_Position())); // 电机数据转现实坐标系
        if (temp_pitch < -180.0f)
            temp_pitch += 360.0f;
    }
    else if (Motor_Pitch_B.Get_Now_Angle() < Motor_Pitch_B.Get_Zero_Position())
    {
        temp_pitch = (Motor_Pitch_B.Get_Zero_Position() - Motor_Pitch_B.Get_Now_Angle());
        if (temp_pitch > 180.0f)
            temp_pitch -= 360.0f;
    }
    Set_True_Angle_Pitch_B(-temp_pitch);
    Motor_Pitch_B.Set_Transform_Angle(Get_True_Angle_Pitch_B());
    Motor_Pitch_B.Set_Transform_Omega(Motor_Pitch_B.Get_Now_Omega_Angle());
    Motor_Pitch_B.Set_Transform_Torque(-Motor_Pitch_B.Get_Now_Torque());
}
void Class_Gimbal::Control_Update_Main()
{
    int invert_flag = 0;
    float temp_error = 0, temp_min = 0, pre_angle = 0, pre_omega = 0;

    temp_error = Get_Target_Yaw_Angle() - Boardc_BMI.Get_Angle_Yaw() - invert_flag * 180;
    while (temp_error > 360.0f)
        temp_error -= 360.0f;
    while (temp_error < 0.0f)
        temp_error += 360.0f;
    if (fabs(temp_error) < (360.0f - fabs(temp_error)))
        temp_min = fabs(temp_error);
    else
        temp_min = 360.0f - fabs(temp_error);
    if (temp_min > 90.0f)
    {
        invert_flag = !invert_flag;
        // 重新计算误差
        temp_error = Get_Target_Yaw_Angle() - Boardc_BMI.Get_Angle_Yaw() - invert_flag * 180.0f;
    }

    if (temp_error > 180.0f)
        temp_error -= 360.0f;
    else if (temp_error < -180.0f)
        temp_error += 360.0f;

    if (temp_error > -0.01f && temp_error < 0.01f)
        temp_error = 0; // 漂移限制

    Set_Target_Yaw_Angle(Boardc_BMI.Get_Angle_Yaw() + temp_error);
    Motor_Main_Yaw.Set_Transform_Angle(Boardc_BMI.Get_Angle_Yaw());
    Motor_Main_Yaw.Set_Transform_Omega(Boardc_BMI.Get_Gyro_Yaw());

    pre_angle = Boardc_BMI.Get_Angle_Yaw();
    pre_omega = Boardc_BMI.Get_Gyro_Yaw();
}

void Class_Gimbal::Yaw_Angle_Limit(Enum_Motor_Yaw_Type Motor_Yaw_Type)
{
    volatile int Motor_Type = Motor_Yaw_Type;
    switch (Motor_Type)
    {
    case (Yaw_A):
    {
        if (MiniPC->Get_Rx_Yaw_Angle_A() < -20.f && MiniPC->Get_Rx_Yaw_Angle_A() > -160.f)
            MiniPC->Set_Auto_Limit_Status_A(Auto_Limit_Status_DISABLE);
        else
            MiniPC->Set_Auto_Limit_Status_A(Auto_Limit_Status_ENABLE);
        break;
    }
    case (Yaw_B):
    {
        if (MiniPC->Get_Rx_Yaw_Angle_B() > 20.f && MiniPC->Get_Rx_Yaw_Angle_B() < 160.f)
            MiniPC->Set_Auto_Limit_Status_B(Auto_Limit_Status_DISABLE);
        else
            MiniPC->Set_Auto_Limit_Status_B(Auto_Limit_Status_ENABLE);
        break;
    }
    }
}

void Class_Gimbal::MiniPC_Update()
{
    MiniPC->Set_Gimbal_Now_Pitch_Angle_A(Get_True_Angle_Pitch_A());
    MiniPC->Set_Gimbal_Now_Pitch_Angle_B(Get_True_Angle_Pitch_B());
    MiniPC->Set_Gimbal_Now_Yaw_Angle(Get_True_Angle_Yaw_Main());
    MiniPC->Set_Gimbal_Now_Yaw_Angle_A(Get_True_Angle_Yaw_A());
    MiniPC->Set_Gimbal_Now_Yaw_Angle_B(Get_True_Angle_Yaw_B());
}
float Gravity_Compensate = 200.f;
void Class_Gimbal::PID_Update()
{
    Motor_Pitch_A.Set_Out(-Motor_Pitch_A.Get_Out() - Gravity_Compensate);

    // Motor_Pitch_B.t = DWT_GetTimeline_us();
    // Motor_Pitch_B.dt= (Motor_Pitch_B.t - Motor_Pitch_B.pre_t) / 1000000.f;
    // if(Motor_Pitch_B.dt < 0)Motor_Pitch_B.dt = Motor_Pitch_B.t + 0xFFFFFFFF - Motor_Pitch_B.pre_t;
    // Motor_Pitch_B.Set_Now_Omega_Angle((Motor_Pitch_B.Get_Now_Angle() - Motor_Pitch_B.Get_Pre_Angle()) /Motor_Pitch_B.dt); //angle
    // Motor_Pitch_B.pre_t = Motor_Pitch_B.t;
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
