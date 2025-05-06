/**
 * @file crt_chassis.cpp
 * @author lez by wanghongxi
 * @brief 底盘
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
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

    //斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.004f,0.008f);
    //斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.004f,0.008f);
    //斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);

    #ifdef FLYING_SLOPE
    
    Boardc_BMI.Init();
    Flying_Slope.Init(&Motor_Wheel[0],&Motor_Wheel[1],&Motor_Wheel[2],&Motor_Wheel[3]);         //轮序不对应

    #endif

    #ifdef POWER_LIMIT_JH
        
    Power_Limit.Init();

    #endif

    #ifdef POWER_LIMIT
    //超级电容初始化
    PowerControl_FSM.Init(2,0);
    PowerControl_FSM.Supercap = &Supercap;
    Supercap.Init(&hcan1,45);

    #endif

    //电机PID批量初始化
    // for (int i = 0; i < 4; i++)
    // {
    //     Motor_Wheel[i].PID_Omega.Init(1500.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    // }
    Motor_Wheel[0].PID_Omega.Init(3500.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[0].Get_Output_Max(), Motor_Wheel[0].Get_Output_Max());
    Motor_Wheel[1].PID_Omega.Init(3500.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[1].Get_Output_Max(), Motor_Wheel[1].Get_Output_Max());
    Motor_Wheel[2].PID_Omega.Init(3500.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[2].Get_Output_Max(), Motor_Wheel[2].Get_Output_Max());
    Motor_Wheel[3].PID_Omega.Init(3500.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[3].Get_Output_Max(), Motor_Wheel[3].Get_Output_Max());

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
float temp_test_1,temp_test_2,temp_test_3,temp_test_4;
void Class_Tricycle_Chassis::Speed_Resolution(){
    //获取当前速度值，用于速度解算初始值获取
    switch (Chassis_Control_Type)
    {
        case (Chassis_Control_Type_DISABLE):
        {
            //底盘失能 四轮子无力
            for (int i = 0; i < 4; i++)
            {
                Motor_Wheel[i].Disable();
            }            
        }
        break;
		case (Chassis_Control_Type_SPIN_Positive) :
        case (Chassis_Control_Type_SPIN_NePositive) :
        case (Chassis_Control_Type_FLLOW):
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
                Math_Constrain(&Target_Omega, -Omega_Max, Omega_Max);
            }

            #ifdef SPEED_SLOPE
            //速度换算，正运动学分解
            float motor1_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
            float motor2_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
            float motor3_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
            float motor4_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
            #else
            //速度换算，正运动学分解
            float motor1_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            float motor2_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            float motor3_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            float motor4_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            #endif            
            //线速度 cm/s  转角速度  RAD 
            float motor1_temp_rad = motor1_temp_linear_vel * VEL2RAD;
            float motor2_temp_rad = motor2_temp_linear_vel * VEL2RAD;
            float motor3_temp_rad = motor3_temp_linear_vel * VEL2RAD;
            float motor4_temp_rad = motor4_temp_linear_vel * VEL2RAD;
            //角速度*减速比  设定目标 直接给到电机输出轴        根据结算对应车轮编号
            Motor_Wheel[0].Set_Target_Omega_Radian(  motor4_temp_rad);
            Motor_Wheel[1].Set_Target_Omega_Radian(- motor3_temp_rad);
            Motor_Wheel[2].Set_Target_Omega_Radian(- motor1_temp_rad);
            Motor_Wheel[3].Set_Target_Omega_Radian(  motor2_temp_rad);

            //各个电机具体PID
            for (int i = 0; i < 4; i++){
                Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
            }
        }
        break;
    }   
}


/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float Power_Limit_K = 1.0f;
extern float Chassis_Power;
void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
    #ifdef SPEED_SLOPE

    //斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

    #endif
    //速度解算
    Speed_Resolution();

    #ifdef FLYING_SLOPE

    if(Get_Chassis_Control_Type() != Chassis_Control_Type_DISABLE){
        Flying_Slope.Transform_Angle();
        Flying_Slope.TIM_Calcualte_Feekback();
        Flying_Slope.Output();
    }

    #endif

    if(Chassis_Control_Type == Chassis_Control_Type_DISABLE){
        return;         //失能情况下不跑功率限制，防止出错
    }

    #ifdef POWER_LIMIT

    //Supercap.Set_Now_Power(Referee->Get_Chassis_Power());
    
    if(Referee->Get_Referee_Status()==Referee_Status_DISABLE){
        Power_Limit.Set_Power_Limit(45.0f);
    }
    else
    {
        PowerControl_FSM.Set_Sprint_Status(Sprint_Status);
        PowerControl_FSM.Set_Chassis_Max_Power(Referee->Get_Chassis_Power_Max());
        PowerControl_FSM.Set_Chassis_Buffer(Referee->Get_Chassis_Energy_Buffer());
        PowerControl_FSM.Reload_TIM_Status_PeriodElapsedCallback();
        Power_Limit.Set_Power_Limit(PowerControl_FSM.Get_PowerLimit_Output());
    }
    
    Power_Limit.Set_Chassis_Buffer(Referee->Get_Chassis_Energy_Buffer());

    if(Supercap.Get_Supercap_Status()==Disconnected)
        Power_Limit.Set_Supercap_Enegry(0.0f);
    else
        Power_Limit.Set_Supercap_Enegry(Supercap.Get_Stored_Energy());
    
    Power_Limit.TIM_Adjust_PeriodElapsedCallback(Motor_Wheel);  //功率限制算法

    Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() + PowerControl_FSM.Get_Buffer_Power() + 5.0f);
    Supercap.TIM_Supercap_PeriodElapsedCallback();          //向超电发送信息

    #elif defined (POWER_LIMIT_JH)
    float Chassis_Buffer = Referee->Get_Chassis_Energy_Buffer();
    
    Power_Management.Buffer_Power = (sqrt(Chassis_Buffer)-sqrt(Power_Management.Min_Buffer))*Power_Management.Buffer_K;
	Math_Constrain(&Power_Management.Buffer_Power,-70.0f,45.0f);

    Power_Management.Max_Power = Referee->Get_Chassis_Power_Max() + Power_Management.Buffer_Power;
    Power_Management.Actual_Power = Chassis_Power;//Referee->Get_Chassis_Power();
    Power_Management.Total_error = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        Power_Management.Motor_Data[i].feedback_omega = Motor_Wheel[i].Get_Now_Omega_Radian() * RAD_TO_RPM * Motor_Wheel[i].Get_Gearbox_Rate();
        Power_Management.Motor_Data[i].feedback_torque = Motor_Wheel[i].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
        Power_Management.Motor_Data[i].torque = Motor_Wheel[i].PID_Omega.Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
        Power_Management.Motor_Data[i].pid_output = Motor_Wheel[i].PID_Omega.Get_Out();

        Power_Management.Motor_Data[i].Target_error = fabs(Motor_Wheel[i].Get_Target_Omega_Radian() - Motor_Wheel[i].Get_Now_Omega_Radian());
        Power_Management.Total_error += Power_Management.Motor_Data[i].Target_error;
    }

      Power_Limit.Power_Task(Power_Management);

    // for (int i = 0; i < 4; i++)
    // {
    //     Motor_Wheel[i].Set_Out(Power_Management.Motor_Data[i].output);
    //     Motor_Wheel[i].Output();
    // }

    
    //Power_Limit.TIM_Adjust_PeriodElapsedCallback(Motor_Wheel);  //功率限制算法


    /****************************超级电容***********************************/
    // Supercap.Set_Now_Power(Referee->Get_Chassis_Power());
    // if(Referee->Get_Referee_Status()==Referee_Status_DISABLE)
    //     Supercap.Set_Limit_Power(45.0f);
    // else
    // {
    //     float offset;
    //     offset = (Referee->Get_Chassis_Energy_Buffer()-20.0f)/4;
    //     Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() + offset);
    // }
        
    // Supercap.TIM_Supercap_PeriodElapsedCallback();

    // /*************************功率限制策略*******************************/
    // if(__Sprint_Status==Sprint_Status_ENABLE)
    // {
    //     //功率限制  
    //     Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max()*1.5f);
    // }
    // else
    // {
    //     Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max());
    // }
    // //Power_Limit.Set_Power_Limit(45.0f);
    // Power_Limit.Set_Motor(Motor_Wheel);   //添加四个电机的控制电流和当前转速
    // Power_Limit.Set_Chassis_Buffer(Referee->Get_Chassis_Energy_Buffer());

    // if(Supercap.Get_Supercap_Status()==Supercap_Status_DISABLE)
    //     Power_Limit.Set_Supercap_Enegry(0.0f);
    // else
    //     Power_Limit.Set_Supercap_Enegry(Supercap.Get_Stored_Energy());
    
    // Power_Limit.TIM_Adjust_PeriodElapsedCallback(Motor_Wheel);  //功率限制算法

    #endif
}

void Class_PowerControl_FSM::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
        case(0):    //超电启用状态
            if(Sprint_Status == Sprint_Status_ENABLE && Supercap->Get_Supercap_Status() != Disconnected){
                Buffer_Power = (Chassis_Buffer - Chassis_Buffer_Min) * Chassis_Buffer_Kp;
                Math_Constrain(&Buffer_Power,-45.0f,45.0f);
                PowerLimit_Output = Chassis_Max_Power + Supercap->Get_Buffer_Power() + Buffer_Power;
            }
            else{
                Set_Status(1);
            }
            
        break;

        case(1):    //使用裁判系统缓冲能量状态
            if(Sprint_Status == Sprint_Status_ENABLE && Supercap->Get_Supercap_Status() != Disconnected){
                Set_Status(0);
            }

            Buffer_Power = (Chassis_Buffer - Chassis_Buffer_Min) * Chassis_Buffer_Kp;
            Math_Constrain(&Buffer_Power,-45.0f,45.0f);
            PowerLimit_Output = Chassis_Max_Power + Buffer_Power;

        break;

        case(2):        //裁判系统低缓冲能量状态
            

        break;

    default:
        break;
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
