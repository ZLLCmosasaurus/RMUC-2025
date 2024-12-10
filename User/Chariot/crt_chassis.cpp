/* Includes ------------------------------------------------------------------*/

#include "crt_chassis.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
#define SPEED_SLOPE
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

//    //斜坡函数加减速速度X  控制周期1ms
//    Slope_Velocity_X.Init(0.006f,0.008f);
//    //斜坡函数加减速速度Y  控制周期1ms
//    Slope_Velocity_Y.Init(0.006f,0.008f);
//    //斜坡函数加减速角速度
//    Slope_Omega.Init(0.02f, 0.025f);
    //斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.006f,0.012f);
    //斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.006f,0.012f);
    //斜坡函数加减速角速度
    Slope_Omega.Init(0.015f, 0.025f);

    #ifdef POWER_LIMIT
    //超级电容初始化
    Supercap.Init(&hcan2,45);
    #endif

    
    //电机PID批量初始化
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].PID_Omega.Init(800.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    }

    //轮向电机ID初始化
    Motor_Wheel[0].Init(&hcan1, DJI_Motor_ID_0x205,DJI_Motor_Control_Method_OMEGA,3591.0f/187.0f);
    Motor_Wheel[1].Init(&hcan1, DJI_Motor_ID_0x202,DJI_Motor_Control_Method_OMEGA,3591.0f/187.0f);
    Motor_Wheel[2].Init(&hcan1, DJI_Motor_ID_0x206,DJI_Motor_Control_Method_OMEGA,3591.0f/187.0f);
    Motor_Wheel[3].Init(&hcan1, DJI_Motor_ID_0x204,DJI_Motor_Control_Method_OMEGA,3591.0f/187.0f);
    
}


/**
 * @brief 全向轮速度解算
 *
 */
#define Max_Omega 31.42f//rpm = 300
float temp_test_1,temp_test_2,temp_test_3,temp_test_4;
void Class_Tricycle_Chassis::Speed_Resolution(){
    //获取当前速度值，用于速度解算初始值获取
    switch (Chassis_Control_Type)
    {
        case (Chassis_Control_Type_DISABLE):
        {
            //底盘失能 四轮子自锁
            for (int i = 0; i < 4; i++)
            {
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);     
                Motor_Wheel[i].Set_Target_Torque(0.0f);
            }            
        }
        break;
		case (Chassis_Control_Type_SPIN) :
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
                Math_Constrain(Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
            }
            if (Velocity_Y_Max != 0)
            {
                Math_Constrain(Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
            }
            if (Omega_Max != 0)
            {
                Math_Constrain(Target_Omega, -Omega_Max, Omega_Max);
            }

            #ifdef SPEED_SLOPE 
            //速度换算，正运动学分解
            float motor1_temp_linear_vel = +Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
            float motor2_temp_linear_vel = +Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
            float motor3_temp_linear_vel = +Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
            float motor4_temp_linear_vel = +Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
            #endif
            //速度换算，正运动学分解
            #ifdef Tricycle_Chassis
            float motor1_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            float motor2_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            float motor3_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            float motor4_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            #endif   
            // float motor1_temp_linear_vel = -Target_Velocity_Y + Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            // float motor2_temp_linear_vel = -Target_Velocity_Y - Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            // float motor3_temp_linear_vel = -Target_Velocity_Y - Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
            // float motor4_temp_linear_vel = -Target_Velocity_Y + Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);         
            //线速度 cm/s  转角速度  RAD 
            float motor1_temp_rad = motor1_temp_linear_vel * VEL2RAD;
            float motor2_temp_rad = motor2_temp_linear_vel * VEL2RAD;
            float motor3_temp_rad = motor3_temp_linear_vel * VEL2RAD;
            float motor4_temp_rad = motor4_temp_linear_vel * VEL2RAD;
            //速度限幅
            Math_Constrain(motor1_temp_rad,-Max_Omega,Max_Omega);
            Math_Constrain(motor2_temp_rad,-Max_Omega,Max_Omega);
            Math_Constrain(motor3_temp_rad,-Max_Omega,Max_Omega);
            Math_Constrain(motor4_temp_rad,-Max_Omega,Max_Omega);
             //角速度*减速比  设定目标 直接给到电机输出轴
            Motor_Wheel[0].Set_Target_Omega_Radian(  -motor2_temp_rad);
            Motor_Wheel[1].Set_Target_Omega_Radian(  motor1_temp_rad);
            Motor_Wheel[2].Set_Target_Omega_Radian(  motor3_temp_rad);
            Motor_Wheel[3].Set_Target_Omega_Radian(  -motor4_temp_rad);

        }
        break;
    }   
}


/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float Power_Limit_K = 1.0f;
void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
#ifdef SPEED_SLOPE

    // 斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

#endif
    // 速度解算
    Speed_Resolution();
    // 各个电机具体PID
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
    }

#ifdef POWER_LIMIT

    /****************************超级电容***********************************/
    Supercap.Set_Now_Power(Referee->Get_Chassis_Power());
    if (Referee->Get_Referee_Status() == Referee_Status_DISABLE)
        Supercap.Set_Limit_Power(45.0f);
    else
    {
        float offset;
        offset = (Referee->Get_Chassis_Energy_Buffer() - 20.0f) / 4;
        Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() + offset);
    }

    Supercap.TIM_Supercap_PeriodElapsedCallback();

    /*************************功率限制策略*******************************/
    if (__Sprint_Status == Sprint_Status_ENABLE)
    {
        // 功率限制
        Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max() * 1.5f);
    }
    else
    {
        Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max());
    }
    // Power_Limit.Set_Power_Limit(45.0f);
    Power_Limit.Set_Motor(Motor_Wheel); // 添加四个电机的控制电流和当前转速
    Power_Limit.Set_Chassis_Buffer(Referee->Get_Chassis_Energy_Buffer());

    if (Supercap.Get_Supercap_Status() == Supercap_Status_DISABLE)
        Power_Limit.Set_Supercap_Enegry(0.0f);
    else
        Power_Limit.Set_Supercap_Enegry(Supercap.Get_Stored_Energy());

    Power_Limit.TIM_Adjust_PeriodElapsedCallback(Motor_Wheel); // 功率限制算法

#endif
}
