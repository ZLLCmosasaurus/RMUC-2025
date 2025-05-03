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
#include "arm_math.h"

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
        Out = Out;
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
        //PID_Angle.Set_Target(0.0f);
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
        Set_Out(-PID_Omega.Get_Out() - 1500);
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
    True_YawTotalAngle = IMU->Get_Angle_YawTotal();
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
        //默认开环
        Out = Out;
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
            PID_Omega.Set_Now(True_Gyro_Pitch);

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
        Set_Out(PID_Omega.Get_Out() - 3450 * cosf((True_Angle_Pitch - 1.0f) *3.1415926f/ 180.0f));
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
    case(LK_Motor_InterControl_Method_IncrementAngle):
	{   
        Out =  (Target_Angle - True_Angle_Pitch)*8;
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
    True_Rad_Pitch = -1 * IMU->Get_Rad_Roll();
    True_Gyro_Pitch = -1 * IMU->Get_Gyro_Roll(); 
    True_Angle_Pitch = -1 * IMU->Get_Angle_Roll();
}

void Class_Gimbal_Pitch_Motor_LK6010::Disable()
{
    Set_LK_Motor_Control_Method(LK_Motor_Control_Method_TORQUE);
    Set_Out(0.0f);
    Output();
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
    //250 300
    Motor_Yaw.PID_Angle.Init(0.335000008f, 0.0f, 0.000789800019f, 0.0f, 1.0f, 20.0f,0.0f,0.0f,0.0f,0.001f,0.0f, PID_D_First_ENABLE);
    Motor_Yaw.PID_Angle.PID_D_Filter.Init(-25000.0f,25000.0f,Filter_Fourier_Type_LOWPASS,3,0);
    Motor_Yaw.PID_Angle.Start_D_Fifter();

    Motor_Yaw.PID_Omega.Init(15000.0f, 12000.0f, 0.0f, -0.0f, 15000.0f, 20000.0f,0.0f,0.0f,0.0f,0.001f,0.0f);
    Motor_Yaw.PID_Torque.Init(0.78f, 100.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.IMU = &Boardc_BMI;
    Motor_Yaw.Init(&hcan2, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_IMU_ANGLE, -3863);

    //pitch轴电机
    Motor_Pitch.PID_Angle.Init(0.550000012f, 0.0f, 0.00015199995f, 0.0f, 0.800000012, 6.0f * PI, 0.0, 0.0, 0.0, 0.001, 0.0,PID_D_First_ENABLE);
    Motor_Pitch.PID_Angle.PID_D_Filter.Init(-20000.0f,20000.0f,Filter_Fourier_Type_LOWPASS,3,0);
    Motor_Pitch.PID_Angle.Start_D_Fifter();
    Motor_Pitch.PID_Omega.Init(-4950.0f, -19000.0f, 0.0f, 0, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max(),0.0f,0.0f,0.0f,0.001f,0.00001f);
    Motor_Pitch.PID_Torque.Init(0.8f, 100.0f, 0.0f, 0.0f, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());
    Motor_Pitch.IMU = &Boardc_BMI;
    Motor_Pitch.Init(&hcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_IMU_ANGLE, -1361);

    // Motor_Pitch_LK6010.PID_Angle.Init(0.2f, 0.0f, 0.0f, 0.0f, 10.0f, 20.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.05f);

    // Motor_Pitch_LK6010.PID_Omega.Init(2.0f, 0.0f, 0.0f, 0.0f, 2000.0f, 2000.0f,0.0f,0.0f,0.0f,0.001f,0.0f);

    // Motor_Pitch_LK6010.IMU = &Boardc_BMI;
    // Motor_Pitch_LK6010.Init(&hcan1, LK_Motor_ID_0x141, 0);       //默认IMU_ANGLE控制
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
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
        //Motor_Pitch_LK6010.Set_LK_Motor_Control_Method(LK_Motor_InterControl_Method_IncrementAngle);
        
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            //设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);         
        }
        else if((Gimbal_Control_Type == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status()!=MiniPC_Status_DISABLE))
        {   
            Target_Pitch_Angle = MiniPC->Get_Rx_Pitch_Angle();
            Target_Yaw_Angle = MiniPC->Get_Rx_Yaw_Angle();          
        }

        //限制角度范围 处理yaw轴180度问题           不要使用YawTotal，会和上位机自瞄的坐标系冲突
        while((Target_Yaw_Angle-Motor_Yaw.Get_True_Angle_Yaw())>Max_Yaw_Angle)
        {
            Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
        }
        while((Target_Yaw_Angle-Motor_Yaw.Get_True_Angle_Yaw())<-Max_Yaw_Angle)
        {
            Target_Yaw_Angle += (2 * Max_Yaw_Angle);
        }
    
        //pitch限位
        Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

        //设置目标角度 遥控器滤波
        // if(First_Flag){
        //     First_Flag = 0;
        // }
        // else{
        //     Target_Yaw_Angle = 0.4*Pre_Target_Yaw_Angle + 0.6*Target_Yaw_Angle;    
        // }
        // Pre_Target_Yaw_Angle = Target_Yaw_Angle;

        Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
        Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
    }
}


/**
 * @brief TIM定时器中断计算回调函数
 *
 */
int t = 0;
int t2 = 0;
float Target = 0;
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    Output();
    //根据不同c板的放置方式来修改这几个函数
    Motor_Yaw.Transform_Angle();
    Motor_Pitch.Transform_Angle();
    //Motor_Pitch_LK6010.Transform_Angle();

    //测试
    // if(t % 3 == 0){
    //     t2 ++;
    //    Target = 5.0f*sinf((float)t2*PI/50.0f);
    // }
    // Motor_Yaw.Set_Target_Angle(Target);
    // t++;
    //

    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    //Motor_Pitch_LK6010.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
