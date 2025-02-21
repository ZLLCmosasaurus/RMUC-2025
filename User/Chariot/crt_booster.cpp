/**
 * @file crt_booster.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 发射机构电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_booster.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
#ifdef _OLD
/**
 * @brief 定时器处理函数
 * 这是一个模板, 使用时请根据不同处理情况在不同文件内重新定义
 *
 */
void Class_FSM_Heat_Detect::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0):
    {
        //正常状态

        if (abs(Booster->Motor_Friction_Right.Get_Now_Torque()) >= Booster->Friction_Torque_Threshold)
        {
            //大扭矩->检测状态
            Set_Status(1);
        }
        else if (Booster->Booster_Control_Type == Booster_Control_Type_DISABLE)
        {
            //停机->停机状态
            Set_Status(3);
        }
    }
    break;
    case (1):
    {
        //发射嫌疑状态

        if (Status[Now_Status_Serial].Time >= 15)
        {
            //长时间大扭矩->确认是发射了
            Set_Status(2);
        }
    }
    break;
    case (2):
    {
        //发射完成状态->加上热量进入下一轮检测

        Heat += 10.0f;
        Set_Status(0);
    }
    break;
    case (3):
    {
        //停机状态

        if (abs(Booster->Motor_Friction_Right.Get_Now_Omega_Radian()) >= Booster->Friction_Omega_Threshold)
        {
            //开机了->正常状态
            Set_Status(0);
        }
    }
    break;
    }

    //热量冷却到0
    if (Heat > 0)
    {
        //Heat -= Booster->Referee->Get_Booster_17mm_1_Heat_CD() / 1000.0f;
    }
    else
    {
        Heat = 0;
    }
}
#endif

/**
 * @brief 卡弹策略有限自动机
 *
 */
void Class_FSM_Antijamming::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
        case (0):
        {
            //正常状态
            Booster->Output();

            if (abs(Booster->Motor_Driver.Get_Now_Torque()) >= Booster->Driver_Torque_Threshold)
            {
                //大扭矩->卡弹嫌疑状态
                Set_Status(1);
            }
        }
        break;
        case (1):
        {
            //卡弹嫌疑状态
            Booster->Output();

            if (Status[Now_Status_Serial].Time >= 500) 
            {
                //第一次进入卡弹 反转到上一格
                // Booster->Drvier_Angle += 2.0f * PI / 6.0f;
                //长时间大扭矩->卡弹反应状态
                Set_Status(2);
            }
            else if (abs(Booster->Motor_Driver.Get_Now_Torque()) < Booster->Driver_Torque_Threshold)
            {
                //短时间大扭矩->正常状态
                Set_Status(0);
            }
        }
        break;
        case (2):
        {
            //卡弹反应状态->准备卡弹处理
            Booster->Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Drvier_Angle = Booster->Motor_Driver.Get_Now_Radian() + PI / 9.0f; //回退20度
            Booster->Motor_Driver.Set_Target_Radian(Booster->Drvier_Angle);
            Set_Status(3);
        }
        break;
        case (3):
        {
            //卡弹处理状态

            if (Status[Now_Status_Serial].Time >= 100)
            {
                //Booster->Drvier_Angle = Booster->Motor_Driver.Get_Now_Radian() - PI / 9.0f; //前进20度
                Booster->Drvier_Angle = Booster->Drvier_Angle - PI / 9.0f; //前进20度
                Booster->Motor_Driver.Set_Target_Radian(Booster->Drvier_Angle);
                Set_Status(4);
            }
			// if(Status[Now_Status_Serial].Time >= 400)
			// {
			// 	Set_Status(0);
			// }
        }
        break;
        case (4):
        {
            if(Status[Now_Status_Serial].Time >= 50)
			{
				Set_Status(0);
			}
        }
        break;
        }
}
    

void Class_Fric_Motor::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环扭矩控制
        Out = Target_Torque / Torque_Max * Output_Max;
    }
    break;
    case (DJI_Motor_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega_Rpm);
        PID_Omega.Set_Now(Data.Now_Omega_Rpm);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    default:
    {
        Out = 0.0f;
    }
    break;
    }
    Output();
}
int16_t Target_Speed = 500;
/**
 * @brief 发射机构初始化
 *
 */
void Class_Booster::Init()
{
    //正常状态, 发射嫌疑状态, 发射完成状态, 停机状态
    // FSM_Heat_Detect.Booster = this;
    // FSM_Heat_Detect.Init(3, 3);

    //正常状态, 卡弹嫌疑状态, 卡弹反应状态, 卡弹处理状态, 卡弹处理反应状态
    FSM_Antijamming.Booster = this;
    FSM_Antijamming.Init(5, 0);

    //拨弹盘电机(需要从新调更新参数)DJI_motor_3508 0X201
    Motor_Driver.PID_Angle.Init(80.0f, 0.8f, 1.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Driver.PID_Omega.Init(3000.0f, 40.0f, 0.0f, 0.0f, Motor_Driver.Get_Output_Max(), Motor_Driver.Get_Output_Max());
    Motor_Driver.Init(&hcan2, DJI_Motor_ID_0x207, DJI_Motor_Control_Method_OMEGA,50.895f);

    //4*摩擦轮初始化
    Fric[0].Init(&hcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 1.0f);
    Fric[0].PID_Omega.Init(10.0f, 0.05f, 0.0f, 0.0f, 2000.0f,11000.0f);

    Fric[1].Init(&hcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, 1.0f);
    Fric[1].PID_Omega.Init(10.0f, 0.05f, 0.0f, 0.0f, 2000.0f,11000.0f);

    Fric[2].Init(&hcan1, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA, 1.0f);
    Fric[2].PID_Omega.Init(10.0f, 0.05f, 0.0f, 0.0f, 2000.0f,11000.0f);

    Fric[3].Init(&hcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_OMEGA, 1.0f);
    Fric[3].PID_Omega.Init(10.0f, 0.05f, 0.0f, 0.0f, 2000.0f,11000.0f);
}

/**
 * @brief 输出到电机
 *
 */
void Class_Booster::Output()
{
    //控制拨弹轮
    //枪口超热量的简单处理方法
    // if((Referee->Get_Booster_42mm_Heat()>Referee->Get_Booster_42mm_Heat_Max()*0.9f)&&Booster_Control_Type!=Booster_Control_Type_DISABLE)
    // {
    //     Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
    // }


    switch (Booster_Control_Type)
    {
        case (Booster_Control_Type_DISABLE):
        {
            // 发射机构失能
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
            Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
            Motor_Driver.Set_Out(0.0f);

            for (auto i = 0; i < 4; i++)
            {
                Fric[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
                Fric[i].Set_Target_Torque(0.0f);
            }
            
            // 关闭摩擦轮
            Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
        }
        break;
        case (Booster_Control_Type_CEASEFIRE):
        {
            for (auto i = 0; i < 4; i++)
            {
                Fric[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            }
            // 停火
            if (Motor_Driver.Get_Control_Method() == DJI_Motor_Control_Method_ANGLE)
            {
                //Motor_Driver.Set_Target_Angle(Motor_Driver.Get_Now_Angle());
            }
            else if (Motor_Driver.Get_Control_Method() == DJI_Motor_Control_Method_OMEGA)
            {
                Motor_Driver.Set_Target_Omega_Radian(0.0f);
            }
        }
        break;
        case (Booster_Control_Type_SINGLE):
        {
            // 单发模式
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            for (auto i = 0; i < 4; i++)
            {
                Fric[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            }

            Drvier_Angle -= 2.0f * PI / 6.0f;
            Motor_Driver.Set_Target_Radian(Drvier_Angle);

            //点一发立刻停火
            Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
        }
        break;
        #ifdef _OLD
        case (Booster_Control_Type_MULTI):
        {
            // 连发模式
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);


            Drvier_Angle -= 2.0f * PI / 6.0f * 2.0f; //两连发
            Motor_Driver.Set_Target_Radian(Drvier_Angle);


            //点一发立刻停火
            Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
        }
        break;
        case (Booster_Control_Type_REPEATED):
        {
            // 连发模式
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            // 根据冷却计算拨弹盘默认速度, 此速度下与冷却均衡
            // Default_Driver_Omega = Referee->Get_Booster_17mm_1_Heat_CD() / 10.0f / 8.0f * 2.0f * PI;

            // 热量控制
            if (abs(Driver_Omega) <= abs(Default_Driver_Omega))
            {
                Motor_Driver.Set_Target_Omega_Radian(Driver_Omega);
            }
            else
            {
                float tmp_omega;
                // tmp_omega = (Default_Driver_Omega - Driver_Omega) / Referee->Get_Booster_17mm_1_Heat_Max() * (FSM_Heat_Detect.Heat + 30.0f) + Driver_Omega;
                Motor_Driver.Set_Target_Omega_Radian(tmp_omega);
            }
        }
        break;  
        #endif
    }

    //控制摩擦轮
    if(Friction_Control_Type != Friction_Control_Type_DISABLE)
    {
        Fric[0].Set_Target_Omega_Rpm(Fric_Low_Rpm);
        Fric[1].Set_Target_Omega_Rpm(Fric_High_Rpm);
        Fric[2].Set_Target_Omega_Rpm(-Fric_High_Rpm);
        Fric[3].Set_Target_Omega_Rpm(-Fric_Low_Rpm);
    }
    else
    {
        Fric[0].Set_Target_Omega_Rpm(0);
        Fric[1].Set_Target_Omega_Rpm(0);
        Fric[2].Set_Target_Omega_Rpm(0);
        Fric[3].Set_Target_Omega_Rpm(0);
    }
}

/**
 * @brief 定时器计算函数
 *
 */
int16_t Test_Torque = 0;float Test_Target_Driver_Angle = 0.0f;float Test_Actual_Driver_Angle = 0.0f;
int16_t Test_Fric1_output,Test_Fric2_output,Test_Fric3_output,Test_Fric4_output;
void Class_Booster::TIM_Calculate_PeriodElapsedCallback()
{

    // 无需裁判系统的热量控制计算
    // FSM_Heat_Detect.Reload_TIM_Status_PeriodElapsedCallback();
    // 卡弹处理
    FSM_Antijamming.Reload_TIM_Status_PeriodElapsedCallback();
     Output();
    Motor_Driver.TIM_PID_PeriodElapsedCallback();
    for (auto i = 0; i < 4; i++)
    {
        Fric[i].TIM_PID_PeriodElapsedCallback();
    }
    Test_Torque = Motor_Driver.Get_Now_Torque();
    Test_Target_Driver_Angle = Get_Drvier_Angle() * 180.0f / PI;
    Test_Actual_Driver_Angle = Motor_Driver.Get_Now_Angle();
    Test_Fric1_output = -Fric[0].Get_Out();
    Test_Fric2_output = -Fric[1].Get_Out();
    Test_Fric3_output = Fric[2].Get_Out();
    Test_Fric4_output = Fric[3].Get_Out();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
