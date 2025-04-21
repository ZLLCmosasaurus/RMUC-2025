/**
 * @file crt_booster.cpp
 * @author lez by wanghongxi
 * @brief 发射机构
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_booster.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/


void Class_Booster::can_shoot(int shoot_count)
{
    shoot_count += 1;//预留一发余地
    // current_heat=Referee->Get_Booster_17mm_1_Heat();//当前热量
    max_heat_capacity =Referee->Get_Booster_17mm_1_Heat_Max();//最大热量
    remain_heat = max_heat_capacity - current_heat;//剩余热量

    if (remain_heat<=heat_per_bullet*shoot_count)//剩余热量小于单发子弹热量*发射数量
    {
        shoot_permission=0;//不允许发射
    }
    else 
    {
        shoot_permission=1;
    }
}
void Class_Booster::if_shoot_complete()
{
    static uint8_t flag__;
    flag__++;
    if(Shoot_Count>=tmp_shoot_count_target)//发射数量大于目标数量
    {
        Shoot_Count=0;//发射数量清零
        tmp_shoot_count_target=0;//目标发射数量清零
        booster_shoot_flag=0;//发射状态置为空闲
        flag__==0;
    }
    else if(Shoot_Count<tmp_shoot_count_target)//发射数量小于目标数量
    {
        booster_shoot_flag=1;//发射状态置为发射
        flag__++;
        if(flag__>=1000)
        {
            Shoot_Count=0;//发射数量清零
            tmp_shoot_count_target=0;//目标发射数量清零
            flag__==0;
        }
    }
}
/**
 * @brief 定时器处理函数
 * 这是一个模板, 使用时请根据不同处理情况在不同文件内重新定义
 *
 */

void Class_FSM_Heat_Detect::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0):
    {
        // 正常状态

        if (abs(Booster->Motor_Friction_Right.Get_Now_Torque()) >= Booster->Friction_Torque_Threshold)
        {
            // 大扭矩->检测状态
            Set_Status(1);
        }
        else if (Booster->Booster_Control_Type == Booster_Control_Type_DISABLE)
        {
            // 停机->停机状态
            Set_Status(3);
        }
    }
    break;
    case (1):
    {
        // 发射嫌疑状态
        if (abs(Booster->Motor_Friction_Right.Get_Now_Torque()) <= Booster->Friction_Torque_Threshold)
        {
            // 大扭矩->检测状态
            Set_Status(0);
        }
        if (Status[Now_Status_Serial].Time >= 10)
        {
            // 长时间大扭矩->确认是发射了
            Set_Status(2);
        }
    }
    break;
    case (2):
    {
        // 发射完成状态->加上热量进入下一轮检测

        Heat += 10.0f;
        Set_Status(0);
        Booster->Shoot_Count++;//发射数量加一
    }
    break;
    case (3):
    {
        // 停机状态

        if (abs(Booster->Motor_Friction_Right.Get_Now_Omega_Radian()) >= Booster->Friction_Omega_Threshold)
        {
            // 开机了->正常状态
            Set_Status(0);
        }
    }
    break;
    }

    // 热量冷却到0
    if (Heat > 0)
    {
        // Heat -= Booster->Referee->Get_Booster_17mm_1_Heat_CD() / 1000.0f;
    }
    else
    {
        Heat = 0;
    }
}

/**
 * @brief 卡弹策略有限自动机
 *
 */
void Class_FSM_Antijamming::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0):
    {
        // 正常状态
        Booster->Output();

        if (abs(Booster->Motor_Driver.Get_Now_Torque()) >= Booster->Driver_Torque_Threshold)
        {
            // 大扭矩->卡弹嫌疑状态
            Set_Status(1);
        }
    }
    break;
    case (1):
    {
        // 卡弹嫌疑状态
        Booster->Output();

        if (Status[Now_Status_Serial].Time >= 100)
        {
            // 长时间大扭矩->卡弹反应状态
            Set_Status(2);
        }
        else if (abs(Booster->Motor_Driver.Get_Now_Torque()) < Booster->Driver_Torque_Threshold)
        {
            // 短时间大扭矩->正常状态
            Set_Status(0);
        }
    }
    break;
    case (2):
    {
        // 卡弹反应状态->准备卡弹处理
        Booster->Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Drvier_Angle = Booster->Motor_Driver.Get_Now_Radian() - PI / 12.0f;
        Booster->Motor_Driver.Set_Target_Radian(Booster->Drvier_Angle);
        Set_Status(3);
    }
    break;
    case (3):
    {
        // 卡弹处理状态

        if (Status[Now_Status_Serial].Time >= 300)
        {
            // 长时间回拨->正常状态
            Set_Status(0);
        }
    }
    break;
    }
}

/**
 * @brief 发射机构初始化
 *
 */
void Class_Booster::Init()
{
    // 正常状态, 发射嫌疑状态, 发射完成状态, 停机状态
    FSM_Heat_Detect.Booster = this;
    FSM_Heat_Detect.Init(3, 3);

    // 正常状态, 卡弹嫌疑状态, 卡弹反应状态, 卡弹处理状态
    FSM_Antijamming.Booster = this;
    FSM_Antijamming.Init(4, 0);

    // 拨弹盘电机
    Motor_Driver.PID_Angle.Init(40.0f, 0.1f, 0.0f, 0.0f, 20.0f * PI, 20.0f * PI);
    Motor_Driver.PID_Omega.Init(6000.0f, 40.0f, 0.0f, 0.0f, Motor_Driver.Get_Output_Max(), Motor_Driver.Get_Output_Max());
    Motor_Driver.Init(&hcan2, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 90);

    // 摩擦轮电机左
    Motor_Friction_Left.PID_Omega.Init(150.0f, 4.0f, 0.2f, 0.0f, 2000.0f, Motor_Friction_Left.Get_Output_Max());
    Motor_Friction_Left.Init(&hcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, 1.0f);

    // 摩擦轮电机右
    Motor_Friction_Right.PID_Omega.Init(150.0f, 4.0f, 0.2f, 0.0f, 2000.0f, Motor_Friction_Right.Get_Output_Max());
    Motor_Friction_Right.Init(&hcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 1.0f);

    #ifdef BULLET_SPEED_PID
    Bullet_Speed.Init(3.0,0,0,0,10,200,0,0,0,0.001,0.3);
    #endif
}

/**
 * @brief 输出到电机
 *
 */
void Class_Booster::Output()
{
can_shoot(1);
	#ifdef BULLET_SPEED_PID
        if (Referee->Get_Referee_Status() == Referee_Status_ENABLE&&Referee->Get_Shoot_Speed()>15&&(Booster_Control_Type==Booster_Control_Type_SINGLE||Booster_Control_Type==Booster_Control_Type_MULTI))
        {
            Bullet_Speed.Set_Now(Referee->Get_Shoot_Speed());
            Bullet_Speed.Set_Target(Target_Bullet_Speed);
            Bullet_Speed.TIM_Adjust_PeriodElapsedCallback();
            Friction_Omega += Bullet_Speed.Get_Out();
        }
#endif
	
    // 控制拨弹轮
    switch (Booster_Control_Type)
    {
    case (Booster_Control_Type_DISABLE):
    {
        // 发射机构失能
        Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        // 关闭摩擦轮
        Set_Friction_Control_Type(Friction_Control_Type_DISABLE);

        Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Friction_Left.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Friction_Right.PID_Angle.Set_Integral_Error(0.0f);

        Motor_Driver.Set_Out(0.0f);
        Motor_Friction_Left.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Right.Set_Target_Omega_Radian(0.0f);
    }
    break;
    case (Booster_Control_Type_CEASEFIRE):
    {
        // 停火
        if (Motor_Driver.Get_Control_Method() == DJI_Motor_Control_Method_ANGLE)
        {
            // Motor_Driver.Set_Target_Angle(Motor_Driver.Get_Now_Angle());
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
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        

            if (shoot_permission == 1 && booster_shoot_flag==0)
            {
                Drvier_Angle += 2.0f * PI / 9.0f;
                Motor_Driver.Set_Target_Radian(Drvier_Angle);
                tmp_shoot_count_target = 1;
            }

        // 点一发立刻停火
        Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
    }
    break;
    case (Booster_Control_Type_MULTI):
    {
        // 五连发模式
        Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        if(booster_shoot_flag==0)
        {
        for(int i=1;i<6;i++)
        {
            can_shoot(i);
            if (shoot_permission==0)
            {
                break;
            }
            else {
            Drvier_Angle += 2.0f * PI / 9.0f;
            tmp_shoot_count_target ++;
            }
        }

        Motor_Driver.Set_Target_Radian(Drvier_Angle);
        }
        // 点五发立刻停火
        Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
    }
    break;
    case  (Booster_Control_Type_REPEATED): 
    {
        // 连发模式
        Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

if(shoot_permission==1)
        {
            Drvier_Angle += 2.0f * PI / 9.0f;
        }

        
        //Motor_Driver.Set_Target_Radian(Drvier_Angle);
       Motor_Driver.Set_Target_Radian(Drvier_Angle);
        // 点五发立刻停火
        Booster_Control_Type = Booster_Control_Type_CEASEFIRE;

    }
    break;
    }

    // 控制摩擦轮
    if (Friction_Control_Type != Friction_Control_Type_DISABLE)
    {

        Motor_Friction_Left.Set_Target_Omega_Radian(Friction_Omega);
        Motor_Friction_Right.Set_Target_Omega_Radian(-Friction_Omega);
    }
    else
    {
        Motor_Friction_Left.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Right.Set_Target_Omega_Radian(0.0f);
    }
}

/**
 * @brief 定时器计算函数
 *
 */
void Class_Booster::TIM_Calculate_PeriodElapsedCallback()
{

    // 无需裁判系统的热量控制计算
     FSM_Heat_Detect.Reload_TIM_Status_PeriodElapsedCallback();
    if_shoot_complete();
    // 需要裁判系统的热量控制计算
    // 已经内置于发射机构的输出函数中
    // 可能导致无裁判系统下无法使用发射机构
    // 现在五连发可以榨干热量而不超限

    // 卡弹处理
    FSM_Antijamming.Reload_TIM_Status_PeriodElapsedCallback();
    
    Output();
   
    Motor_Driver.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Left.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Right.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
