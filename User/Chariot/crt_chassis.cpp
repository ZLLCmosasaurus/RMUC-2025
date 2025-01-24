/**
 * @file crt_chassis.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 底盘电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
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
#ifdef Tricycle_Chassis
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

    #ifdef POWER_LIMIT
    //超级电容初始化
    Supercap.Init(&hcan2,45);
    #endif

    
    //电机PID批量初始化
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].PID_Omega.Init(1500.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    }

    //轮向电机ID初始化
    Motor_Wheel[0].Init(&hcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hcan1, DJI_Motor_ID_0x204);
    
}
#endif
#ifdef Tricycle_Chassis
/**
 * @brief 全向轮速度解算
 *
 */
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
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Wheel[i].PID_Angle.Set_Integral_Error(0.0f);
                Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
                Motor_Wheel[i].Set_Out(0.0f);
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
            //角速度*减速比  设定目标 直接给到电机输出轴
            Motor_Wheel[0].Set_Target_Omega_Radian(  motor2_temp_rad);
            Motor_Wheel[1].Set_Target_Omega_Radian(- motor1_temp_rad);
            Motor_Wheel[2].Set_Target_Omega_Radian(- motor3_temp_rad);
            Motor_Wheel[3].Set_Target_Omega_Radian(  motor4_temp_rad);
            //各个电机具体PID
            for (int i = 0; i < 4; i++){
                Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
            }
            //            Motor_Wheel[0].Set_Target_Omega_Radian(  temp_test_1);
            //            Motor_Wheel[1].Set_Target_Omega_Radian( temp_test_2);
            //            Motor_Wheel[2].Set_Target_Omega_Radian( temp_test_3);
            //            Motor_Wheel[3].Set_Target_Omega_Radian(  temp_test_4);

            // max=find_max();
            // if(max>MAX_MOTOR_SPEED)
            // {
            //     Motor_Wheel[0].Set_Target_Omega(chassis_motor1.target_speed*MAX_MOTOR_SPEED*1.0/max);
            //     chassis_motor2.target_speed=(int)(chassis_motor2.target_speed*MAX_MOTOR_SPEED*1.0/max);
            //     chassis_motor3.target_speed=(int)(chassis_motor3.target_speed*MAX_MOTOR_SPEED*1.0/max);
            //     chassis_motor4.target_speed=(int)(chassis_motor4.target_speed*MAX_MOTOR_SPEED*1.0/max);
            // }
        }
        break;
    }   
}
#endif
#ifdef Tricycle_Chassis
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float Power_Limit_K = 1.0f;
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

    #ifdef POWER_LIMIT
    
    /****************************超级电容***********************************/
    Supercap.Set_Now_Power(Referee->Get_Chassis_Power());
    if(Referee->Get_Referee_Status()==Referee_Status_DISABLE)
        Supercap.Set_Limit_Power(45.0f);
    else
    {
        float offset;
        offset = (Referee->Get_Chassis_Energy_Buffer()-20.0f)/4;
        Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() + offset);
    }
        
    Supercap.TIM_Supercap_PeriodElapsedCallback();

    /*************************功率限制策略*******************************/
    if(__Sprint_Status==Sprint_Status_ENABLE)
    {
        //功率限制  
        Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max()*1.5f);
    }
    else
    {
        Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max());
    }
    //Power_Limit.Set_Power_Limit(45.0f);
    Power_Limit.Set_Motor(Motor_Wheel);   //添加四个电机的控制电流和当前转速
    Power_Limit.Set_Chassis_Buffer(Referee->Get_Chassis_Energy_Buffer());

    if(Supercap.Get_Supercap_Status()==Supercap_Status_DISABLE)
        Power_Limit.Set_Supercap_Enegry(0.0f);
    else
        Power_Limit.Set_Supercap_Enegry(Supercap.Get_Stored_Energy());
    
    Power_Limit.TIM_Adjust_PeriodElapsedCallback(Motor_Wheel);  //功率限制算法

    #endif
}
#endif
#define SPEED_SLOPE
//#define NO_SPEED_SLOPE
float slope_omega = 0.05;
void Class_Streeing_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max, float __Steer_Power_Ratio)
{
    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
    Steer_Power_Ratio = __Steer_Power_Ratio;

    //斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.04f,0.08f);
    //斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.04f,0.08f);
    //斜坡函数加减速角速度
    Slope_Omega.Init(slope_omega, slope_omega);
    
    // 超级电容初始化
    Supercap.Init(&hcan2, 40.0f);
}

void Class_Streeing_Chassis::Speed_Resolution()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        // 底盘失能 四舵轮输出转矩为0
    }
    break;
    case (Chassis_Control_Type_SPIN):
    case (Chassis_Control_Type_FLLOW):
    {
        // 底盘限速
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
        #ifdef NO_SPEED_SLOPE
        // 速度换算，正运动学分解
        // 右下位置的舵轮
        float Wheel_A_Vx = Target_Velocity_X - Target_Omega * R_A * sin(THETA_A);
        float Wheel_A_Vy = Target_Velocity_Y - Target_Omega * R_A * cos(THETA_A);
        // 左下位置的舵轮
        float Wheel_B_Vx = Target_Velocity_X - Target_Omega * R_B * sin(THETA_B);
        float Wheel_B_Vy = Target_Velocity_Y + Target_Omega * R_B * cos(THETA_B);
        // 左上位置的舵轮
        float Wheel_C_Vx = Target_Velocity_X + Target_Omega * R_C * sin(THETA_C);
        float Wheel_C_Vy = Target_Velocity_Y + Target_Omega * R_C * cos(THETA_C);
        // 右上位置的舵轮
        float Wheel_D_Vx = Target_Velocity_X + Target_Omega * R_D * sin(THETA_D);
        float Wheel_D_Vy = Target_Velocity_Y - Target_Omega * R_D * cos(THETA_D);
        #endif
        #ifdef SPEED_SLOPE
        // 速度换算，正运动学分解
        // 右下位置的舵轮
        float Wheel_A_Vx = Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * R_A * sin(THETA_A);
        float Wheel_A_Vy = Slope_Velocity_Y.Get_Out() - Slope_Omega.Get_Out() * R_A * cos(THETA_A);
        // 左下位置的舵轮
        float Wheel_B_Vx = Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * R_B * sin(THETA_B);
        float Wheel_B_Vy = Slope_Velocity_Y.Get_Out() + Slope_Omega.Get_Out() * R_B * cos(THETA_B);
        // 左上位置的舵轮
        float Wheel_C_Vx = Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * R_C * sin(THETA_C);
        float Wheel_C_Vy = Slope_Velocity_Y.Get_Out() + Slope_Omega.Get_Out() * R_C * cos(THETA_C);
        // 右上位置的舵轮
        float Wheel_D_Vx = Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * R_D * sin(THETA_D);
        float Wheel_D_Vy = Slope_Velocity_Y.Get_Out() - Slope_Omega.Get_Out() * R_D * cos(THETA_D);
        #endif
        
        wheel[0].ChassisCoordinate_Angle = My_atan(Wheel_A_Vy, Wheel_A_Vx) * RAD_TO_8191;
        wheel[1].ChassisCoordinate_Angle = My_atan(Wheel_B_Vy, Wheel_B_Vx) * RAD_TO_8191;
        wheel[2].ChassisCoordinate_Angle = My_atan(Wheel_C_Vy, Wheel_C_Vx) * RAD_TO_8191;
        wheel[3].ChassisCoordinate_Angle = My_atan(Wheel_D_Vy, Wheel_D_Vx) * RAD_TO_8191;

        wheel[0].streeing_wheel_speed = sqrt(Square(Wheel_A_Vx) + Square(Wheel_A_Vy));
        wheel[1].streeing_wheel_speed = sqrt(Square(Wheel_B_Vx) + Square(Wheel_B_Vy));
        wheel[2].streeing_wheel_speed = sqrt(Square(Wheel_C_Vx) + Square(Wheel_C_Vy));
        wheel[3].streeing_wheel_speed = sqrt(Square(Wheel_D_Vx) + Square(Wheel_D_Vy));

        for (auto i = 0; i < 4; i++)
        {
            wheel[i].streeing_wheel_omega = wheel[i].streeing_wheel_speed * VEL2RPM;
        }
        
    }
    break;
    }
}
void Class_Streeing_Chassis::AGV_DirectiveMotor_TargetStatus_To_MotorAngle_In_ChassisCoordinate()
{
    // 轮子方向超前坐标方向90°
    // for (auto i = 0; i < 4; i++)
    // {
    //     wheel[i].streeing_wheel_angle = wheel[i].ChassisCoordinate_Angle - PI / 2 / (2 * PI) * 8191;
    // }

    // if (break_mode)
    // {
    //     if (Target_Omega >= 0)
    //     {
    //         wheel[0].streeing_wheel_angle = 8191 * (215) / 360;
    //         wheel[1].streeing_wheel_angle = 8191 * (135) / 360;
    //         wheel[2].streeing_wheel_angle = 8191 * (45) / 360;
    //         wheel[3].streeing_wheel_angle = 8191 * (315) / 360;
    //     }
    //     else
    //     {
    //         wheel[0].streeing_wheel_angle = 8191 * (35) / 360;
    //         wheel[1].streeing_wheel_angle = 8191 * (315) / 360;
    //         wheel[2].streeing_wheel_angle = 8191 * (225) / 360;
    //         wheel[3].streeing_wheel_angle = 8191 * (135) / 360;
    //     }
    // }

    for (auto i = 0; i < 4; i++)
    {
        wheel[i].streeing_wheel_angle = wheel[i].ChassisCoordinate_Angle;
    }
    if (break_mode)
    {
        wheel[0].streeing_wheel_angle = 8191 * (215) / 360;
        wheel[1].streeing_wheel_angle = 8191 * (135) / 360;
        wheel[2].streeing_wheel_angle = 8191 * (45) / 360;
        wheel[3].streeing_wheel_angle = 8191 * (315) / 360;
    }
    for (auto i = 0; i < 4; i++)
    {
        
        if (wheel[i].streeing_wheel_angle > 8191)
            wheel[i].streeing_wheel_angle -= 8191;
        else if (wheel[i].streeing_wheel_angle < 0)
            wheel[i].streeing_wheel_angle += 8191;
    }
}
void Class_Streeing_Chassis::Speed_Limitation()
{
    float temp = 0;
    temp = fabsf(wheel[0].streeing_wheel_omega);
    if (fabsf(wheel[1].streeing_wheel_omega) > temp)
        temp = fabsf(wheel[1].streeing_wheel_omega);
    if (fabsf(wheel[2].streeing_wheel_omega) > temp)
        temp = fabsf(wheel[2].streeing_wheel_omega);
    if (fabsf(wheel[3].streeing_wheel_omega) > temp)
        temp = fabsf(wheel[3].streeing_wheel_omega);
    if (temp > MAX_MOTOR_SPEED)
    {
        for (auto i = 0; i < 4; i++)
        {
            wheel[i].streeing_wheel_omega = (wheel[i].streeing_wheel_omega * MAX_MOTOR_SPEED * 1.0f / temp);
        }
    }
    else
    {
        for (auto i = 0; i < 4; i++)
        {
            wheel[i].streeing_wheel_omega = (wheel[i].streeing_wheel_omega);
        }
    }
}

void Class_Streeing_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
    static uint16_t time_cnt = 0;

    // 1.底盘四舵轮驻车模式
    // 2.随动模式不进行移动状态下，为了正确设置四舵轮角度
    // 3.地盘小陀螺不进行移动状态下，为了正确设置四舵轮角度
    // if ((Target_Velocity_X != 0) && (Target_Velocity_Y != 0))
    // {
    //     break_mode = 0;
    // }
    // else
    // {
    //     break_mode = 1;
    // }
    if ((Target_Velocity_X == 0) && (Target_Velocity_Y == 0) && (Target_Omega == 0))
    {
        break_mode = 1;
    }
    else
    {
        break_mode = 0;
    }
    
    #ifdef SPEED_SLOPE
    //斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
    #endif
    
    Speed_Resolution();
    AGV_DirectiveMotor_TargetStatus_To_MotorAngle_In_ChassisCoordinate();
    Speed_Limitation();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/