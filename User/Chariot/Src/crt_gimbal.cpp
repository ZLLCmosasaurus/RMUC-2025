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
    Motor_Yaw_A.PID_Angle.Init(30.f, 5.0f, 0.0f, 0.0f, Motor_Yaw_A.Get_Output_Max(), Motor_Yaw_A.Get_Output_Max());
    Motor_Yaw_A.PID_Omega.Init(40.0f, 10.0f, 0.0f, 0.0f, 4000, Motor_Yaw_A.Get_Output_Max(), 50, 100);
    Motor_Yaw_A.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Yaw_A.Get_Output_Max(), Motor_Yaw_A.Get_Output_Max());
    Motor_Yaw_A.Init(&hfdcan2, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 2048);

    Motor_Yaw_B.PID_Angle.Init(20.f, 5.0f, 0.0f, 0.0f, Motor_Yaw_B.Get_Output_Max(), Motor_Yaw_B.Get_Output_Max());
    Motor_Yaw_B.PID_Omega.Init(40.0f, 10.0f, 0.0f, 0.0f, 8000, Motor_Yaw_B.Get_Output_Max(), 50, 100);
    Motor_Yaw_B.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Yaw_B.Get_Output_Max(), Motor_Yaw_B.Get_Output_Max());
    Motor_Yaw_B.Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 2048);

    Motor_Main_Yaw.PID_Angle.Init(0.18f, 0.0f, 0.0f, 0.0f, 3, 15);
    Motor_Main_Yaw.PID_Omega.Init(1000.0f, 5.0f, 0.0f, 0.0f, 400.0f, 2048.0f);
    Motor_Main_Yaw.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Main_Yaw.Get_Output_Max(), Motor_Main_Yaw.Get_Output_Max());
    Motor_Main_Yaw.Init(&hfdcan3, LK_Motor_ID_0x141, LK_Motor_Control_Method_ANGLE, 2048);

    // pitch轴电机
    Motor_Pitch_A.PID_Angle.Init(25.f, 2.0f, 0.0f, 0.0f, Motor_Pitch_B.Get_Output_Max(), Motor_Pitch_B.Get_Output_Max());
    Motor_Pitch_A.PID_Omega.Init(60.0f, 10.0f, 0.0f, 0.0f, 8000, Motor_Pitch_A.Get_Output_Max());
    Motor_Pitch_A.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Pitch_A.Get_Output_Max(), Motor_Pitch_A.Get_Output_Max());
    Motor_Pitch_A.Init(&hfdcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE, 3413);

    Motor_Pitch_B.PID_Angle.Init(30.f, 7.0f, 0.0f, 0.0f, Motor_Pitch_B.Get_Output_Max(), Motor_Pitch_B.Get_Output_Max());
    Motor_Pitch_B.PID_Omega.Init(40.0f, 5.0f, 0.0f, 0.0f, 8000, Motor_Pitch_B.Get_Output_Max());
    Motor_Pitch_B.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Pitch_B.Get_Output_Max(), Motor_Pitch_B.Get_Output_Max());
    Motor_Pitch_B.Init(&hfdcan1, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE, 3413);

    Motor_Main_Yaw.Set_Zero_Position(212.308533f);
    Motor_Yaw_A.Set_Zero_Position(294.9169f);
    Motor_Yaw_B.Set_Zero_Position(88.9013f);
    Motor_Pitch_A.Set_Zero_Position(166.157227f);
    Motor_Pitch_B.Set_Zero_Position(262.749023f);
}


/**
 * @brief 输出到电机
 *
 */
float temp_ang_a,temp_ang_b;
float pre_omega_a,pre_omega_b,pre_angle_main;
float last_angle_a,last_angle_b;
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

        A_Cruise_Flag = 0;
        B_Cruise_Flag = 0;

        //MiniPC_Init_Flag;
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

            //Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            // 限制角度

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

            A_Cruise_Flag = 0;
            B_Cruise_Flag = 0;
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            // if(MiniPC->Get_Main_Yaw_Status() == Main_Yaw_Cruise)
            // {
                // A云台控制逻辑
                if (MiniPC->Get_Auto_aim_Status_A() == Auto_aim_Status_DISABLE)
                {
        
                    Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    if(A_Cruise_Flag == 0)//巡航初启动
                    {
                        Motor_Yaw_A.Set_Target_Omega_Angle(CRUISE_SPEED_YAW);
                        Motor_Pitch_A.Set_Target_Omega_Angle(CRUISE_SPEED_PITCH);
                        A_Cruise_Flag = 1;
                        if(pre_omega_a == CRUISE_SPEED_YAW * 1.50f)
                        {
                            Motor_Yaw_A.Set_Target_Omega_Angle(CRUISE_SPEED_YAW);
                        }
                        else if(pre_omega_a == -CRUISE_SPEED_YAW * 1.50f)
                        {
                            Motor_Yaw_A.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW);
                        }
                    }               
                    if (Get_True_Angle_Yaw_A() < -90.f && Get_True_Angle_Yaw_A() > -172.f)
                        Motor_Yaw_A.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW);
                    else if (Get_True_Angle_Yaw_A() <= -10.f && Get_True_Angle_Yaw_A() > -90.0f)
                        Motor_Yaw_A.Set_Target_Omega_Angle(CRUISE_SPEED_YAW);

                    if (Get_True_Angle_Pitch_A() >= 15.0f)
                        Motor_Pitch_A.Set_Target_Omega_Angle(-CRUISE_SPEED_PITCH);
                    else if (Get_True_Angle_Pitch_A() <= 0.0f)
                        Motor_Pitch_A.Set_Target_Omega_Angle(CRUISE_SPEED_PITCH);

                    // 云台控制方式
                    if(MiniPC->Get_Rx_Pitch_Angle_A() != 0.0f || MiniPC->Get_Rx_Yaw_Angle_A() != 0.0f)//00正常巡航
                    {
                        A_Invert_Flag = 1;//不用了(有bug)，先置为0，启用置1
                        temp_ang_a = MiniPC->Get_Rx_Yaw_Angle_A();
                    }
                    else
                    {
                        A_Invert_Flag = 0;
                    }
                    if(A_Invert_Flag == 1)
                    {
                        Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                        Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                        Motor_Pitch_A.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_A());//提前预瞄

                        if(temp_ang_a >= last_angle_a || temp_ang_a <= -160.f)//预测位在逆时针方向
                        {                    
                            Motor_Yaw_A.Set_Target_Omega_Angle(CRUISE_SPEED_YAW * 1.50f);
                        }
                        else
                        {
                            Motor_Yaw_A.Set_Target_Omega_Angle(- CRUISE_SPEED_YAW * 1.50f);//预测位在顺时针方向
                        }								
                        A_Cruise_Flag = 0;                  
                    }
                    last_angle_a = Get_True_Angle_Yaw_A();//保留上一帧数据      
                    pre_omega_a = Motor_Yaw_A.Get_Target_Omega_Angle();        
                }
                else if (MiniPC->Get_Auto_aim_Status_A() == Auto_aim_Status_ENABLE)
                {
                    Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                    Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                    Motor_Yaw_A.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle_A());
                    Motor_Pitch_A.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_A());

                    if(MiniPC->Get_Rx_Pitch_Angle_A() == 0.0f && MiniPC->Get_Rx_Yaw_Angle_A() == 0.0f)
                    {
                        Motor_Yaw_A.Set_Target_Angle(Get_True_Angle_Yaw_A());
                        Motor_Pitch_A.Set_Target_Angle(Get_True_Angle_Pitch_A());
                    }

                    A_Cruise_Flag = 0;
                    A_Invert_Flag = 0;
                }
                // B云台控制逻辑
                if (MiniPC->Get_Auto_aim_Status_B() == Auto_aim_Status_DISABLE)
                {
                    // 云台控制方式
                    Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    if(B_Cruise_Flag == 0)//巡航初启动
                    {
                        Motor_Yaw_B.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW);
                        Motor_Pitch_B.Set_Target_Omega_Angle(CRUISE_SPEED_PITCH);
                        B_Cruise_Flag = 1;
                        if(pre_omega_b == CRUISE_SPEED_YAW * 1.50f)
                        {
                            Motor_Yaw_B.Set_Target_Omega_Angle(CRUISE_SPEED_YAW);
                        }
                        else if(pre_omega_b == -CRUISE_SPEED_YAW * 1.50f)
                        {
                            Motor_Yaw_B.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW);
                        }
                    }

                    if (Get_True_Angle_Yaw_B() > 90.f && Get_True_Angle_Yaw_B() < 172.f)
                    {
                        Motor_Yaw_B.Set_Target_Omega_Angle(CRUISE_SPEED_YAW);
                    }
                    
                    else if (Get_True_Angle_Yaw_B() >= 10.f && Get_True_Angle_Yaw_B() <= 90.0f)
                    {
                        Motor_Yaw_B.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW);
                    }

                    if (Get_True_Angle_Pitch_B() >= 15.0f)
                        Motor_Pitch_B.Set_Target_Omega_Angle(-CRUISE_SPEED_PITCH);
                    else if (Get_True_Angle_Pitch_B() <= 0.0f)
                        Motor_Pitch_B.Set_Target_Omega_Angle(CRUISE_SPEED_PITCH);

                    if(MiniPC->Get_Rx_Pitch_Angle_B() != 0.f || MiniPC->Get_Rx_Yaw_Angle_B() != 0.f)//00巡航
                    {
                        B_Invert_Flag = 1;//不用了(有bug)，先置为0，启用置1
                        temp_ang_b = MiniPC->Get_Rx_Yaw_Angle_B();//预测位
                    }
                    else
                    {
                         B_Invert_Flag = 0;
                    }
                    if(B_Invert_Flag == 1)
                    {
                        Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                        Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                        Motor_Pitch_B.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_B());//提前预瞄
                                        
                        if(temp_ang_b <= last_angle_b || temp_ang_b > 160.f)//预测位在顺时针方向     
                        {
                            Motor_Yaw_B.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW * 1.50f);
                        }
                        else//预测位在逆时针方向
                        {
                            Motor_Yaw_B.Set_Target_Omega_Angle(CRUISE_SPEED_YAW * 1.50f);
                        }
                        B_Cruise_Flag = 0;
                    }
                    last_angle_b = Get_True_Angle_Yaw_B();
                    pre_omega_b = Motor_Yaw_B.Get_Target_Omega_Angle();
                }
                else if (MiniPC->Get_Auto_aim_Status_B() == Auto_aim_Status_ENABLE)//右头自瞄开启
                {
                    Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                    Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                    Motor_Yaw_B.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle_B());
                    Motor_Pitch_B.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_B());

                    if(MiniPC->Get_Rx_Pitch_Angle_B() == 0.f && MiniPC->Get_Rx_Yaw_Angle_B() == 0.f)//
                    {
                        Motor_Yaw_B.Set_Target_Omega_Angle(Get_True_Angle_Yaw_B());
                        Motor_Pitch_B.Set_Target_Omega_Angle(Get_True_Angle_Pitch_B());
                    }

                    B_Cruise_Flag = 0;
                    B_Invert_Flag = 0;
                }
                //大yaw不动
                // Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
                // Motor_Main_Yaw.Set_Target_Angle(Get_Target_Yaw_Angle());
               if(MiniPC->Get_Gimbal_Angular_Velocity_Yaw_Main() != 0.f)
               {
                   Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_OMEGA);
                   Motor_Main_Yaw.Set_Target_Omega_Angle(float(MiniPC->Get_Gimbal_Angular_Velocity_Yaw_Main()/100.f));
                   pre_angle_main = Boardc_BMI.Get_Angle_Yaw();
               }
               else
               {
                   Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
                   Motor_Main_Yaw.Set_Target_Angle(pre_angle_main);
               }
            // }
            // else if(MiniPC->Get_Main_Yaw_Status() == Main_Yaw_Working)
            // {
                    // //左云台
                    // Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                    // Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                    // Motor_Yaw_A.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle_A());
                    // Motor_Pitch_A.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_A());
                    
                    // //右云台
                    // Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                    // Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                    // Motor_Yaw_B.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle_B());
                    // Motor_Pitch_B.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_B());
                    // //大yaw
                    // Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
                    // Motor_Main_Yaw.Set_Target_Angle(Boardc_BMI.Get_Angle_Yaw() + MiniPC->Get_Rx_Angle_Yaw_Main());
            // }
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

    //根据零位的不同来修改这几个函数
    Yaw_Angle_Transform_A();
    Yaw_Angle_Transform_B();
    Pitch_Angle_Transform_A();
    Pitch_Angle_Transform_B();
    Yaw_Angle_Transform_Main();

    MiniPC_Update(); // 上位机数据更新
    //Yaw_Angle_Limit(Yaw_A);
    //Yaw_Angle_Limit(Yaw_B);//都停用

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

    temp_yaw = IMU_Data_A.Yaw - (IMU_Data_A.Yaw - Get_True_Angle_Yaw_A());
    if(temp_yaw > 180.f)
        temp_yaw -= 360.f;
    if(temp_yaw < -180.f)
        temp_yaw += 360.f;
    Set_True_Angle_Yaw_A(temp_yaw);

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
    Motor_Yaw_A.Set_Transform_Omega(IMU_Data_A.Omega_Z);
}
void Class_Gimbal::Yaw_Angle_Transform_B()
{
    float temp_yaw;
    // 如果电机Yaw_B的当前角度大于零位置
    if (Motor_Yaw_B.Get_Now_Angle() > Motor_Yaw_B.Get_Zero_Position())//电机零位校准
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
    temp_yaw = IMU_Data_B.Yaw - (IMU_Data_B.Yaw - Get_True_Angle_Yaw_B());
    if(temp_yaw > 180.f)
        temp_yaw -= 360.f;
    if(temp_yaw < -180.f)
        temp_yaw += 360.f;

    Set_True_Angle_Yaw_B(temp_yaw);

    Motor_Yaw_B.Set_Transform_Angle(Get_True_Angle_Yaw_B());
    Motor_Yaw_B.Set_Transform_Omega(IMU_Data_B.Omega_Z);

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
    Set_True_Angle_Pitch_A(IMU_Data_A.Pitch);

    Motor_Pitch_A.Set_Transform_Angle(Get_True_Angle_Pitch_A());
    Motor_Pitch_A.Set_Transform_Omega(IMU_Data_A.Omega_Y);
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
    Set_True_Angle_Pitch_B(IMU_Data_B.Pitch);
    Motor_Pitch_B.Set_Transform_Angle(Get_True_Angle_Pitch_B());
    Motor_Pitch_B.Set_Transform_Omega(IMU_Data_B.Omega_Y);
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

    Set_Target_Yaw_Angle(Boardc_BMI.Get_Angle_Yaw() + temp_error);
    Motor_Main_Yaw.Set_Transform_Angle(Boardc_BMI.Get_Angle_Yaw());
    Motor_Main_Yaw.Set_Transform_Omega(Boardc_BMI.Get_Gyro_Yaw());

    pre_angle = Boardc_BMI.Get_Angle_Yaw();
    pre_omega = Boardc_BMI.Get_Gyro_Yaw();
}

void Class_Gimbal::Yaw_Angle_Limit(Enum_Motor_Yaw_Type Motor_Yaw_Type)//角度限制（暂时停用）
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

void Class_Gimbal::MiniPC_Update()//上位机数据更新
{
    MiniPC->Set_Gimbal_Now_Pitch_Angle_A(Get_True_Angle_Pitch_A());
    MiniPC->Set_Gimbal_Now_Pitch_Angle_B(Get_True_Angle_Pitch_B());
    MiniPC->Set_Gimbal_Now_Yaw_Angle(Get_True_Angle_Yaw_Main());
    MiniPC->Set_Gimbal_Now_Yaw_Angle_A(Get_True_Angle_Yaw_A());
    MiniPC->Set_Gimbal_Now_Yaw_Angle_B(Get_True_Angle_Yaw_B());
}
float Gravity_Compensate = 0.f;
void Class_Gimbal::PID_Update()
{
    Motor_Pitch_A.Set_Out(-Motor_Pitch_A.Get_Out() - Gravity_Compensate);//PID输出值方向校准（硬件层面问题）

}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
