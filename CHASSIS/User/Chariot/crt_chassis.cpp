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

    // 斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.004f, 0.008f);
    // 斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.004f, 0.008f);
    // 斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);

#ifdef POWER_LIMIT
    // 超级电容初始化
    Supercap.Init(&hcan2, 45);
#endif
// 全向轮类初始化
#ifdef omni_wheel
    // 电机PID批量初始化
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].PID_Omega.Init(1500.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    }

    // 轮向电机ID初始化
    Motor_Wheel[0].Init(&hcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hcan1, DJI_Motor_ID_0x204);

#endif
// 舵轮类初始化
#ifdef steering_wheel

    Agv_Board[0].Init(&hcan1, Agv_BoardA);
    Agv_Board[1].Init(&hcan1, Agv_BoardB);
    Agv_Board[2].Init(&hcan1, Agv_BoardC);
    Agv_Board[3].Init(&hcan1, Agv_BoardD);

#endif
}

/**
 * @brief 速度解算
 *
 */

float temp_test_1, temp_test_2, temp_test_3, temp_test_4;
void Class_Tricycle_Chassis::Speed_Resolution()
{
#ifdef omni_wheel
    // 获取当前速度值，用于速度解算初始值获取
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        // 底盘失能 四轮子自锁
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Wheel[i].PID_Angle.Set_Integral_Error(0.0f);
            Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
            Motor_Wheel[i].Set_Out(0.0f);
        }
    }
    break;
    case (Chassis_Control_Type_SPIN):
    case (Chassis_Control_Type_FLLOW):
    {
        // 底盘四电机模式配置
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        }
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

#ifdef SPEED_SLOPE
        // 速度换算，正运动学分解
        float motor1_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        float motor2_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        float motor3_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        float motor4_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
#else
        // 速度换算，正运动学分解
        float motor1_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X + Target_Omega * (HALF_WIDTH + HALF_LENGTH);
        float motor2_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X - Target_Omega * (HALF_WIDTH + HALF_LENGTH);
        float motor3_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X + Target_Omega * (HALF_WIDTH + HALF_LENGTH);
        float motor4_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X - Target_Omega * (HALF_WIDTH + HALF_LENGTH);
#endif
        // 线速度 cm/s  转角速度  RAD
        float motor1_temp_rad = motor1_temp_linear_vel * VEL2RAD;
        float motor2_temp_rad = motor2_temp_linear_vel * VEL2RAD;
        float motor3_temp_rad = motor3_temp_linear_vel * VEL2RAD;
        float motor4_temp_rad = motor4_temp_linear_vel * VEL2RAD;
        // 角速度*减速比  设定目标 直接给到电机输出轴
        Motor_Wheel[0].Set_Target_Omega_Radian(motor2_temp_rad);
        Motor_Wheel[1].Set_Target_Omega_Radian(-motor1_temp_rad);
        Motor_Wheel[2].Set_Target_Omega_Radian(-motor3_temp_rad);
        Motor_Wheel[3].Set_Target_Omega_Radian(motor4_temp_rad);

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

    // 各个电机具体PID
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
    }
#endif
#ifdef steering_wheel

    //单位为m/s
    float Chassis_Vr_A = Target_Omega * R_A;
    float Chassis_Vr_B = Target_Omega * R_B;
    float Chassis_Vr_C = Target_Omega * R_C;
    float Chassis_Vr_D = Target_Omega * R_D;

    // 计算A轮的速度分量
    float Vy_A = Target_Velocity_Y - Chassis_Vr_A * cos(THETA_A);
    float Vx_A = Target_Velocity_X - Chassis_Vr_A * sin(THETA_A);

    // 计算B轮的速度分量
    float Vy_B = Target_Velocity_Y + Chassis_Vr_B * cos(THETA_B);
    float Vx_B = Target_Velocity_X - Chassis_Vr_B * sin(THETA_B);

    // 计算C轮的速度分量
    float Vy_C = Target_Velocity_Y + Chassis_Vr_C * cos(THETA_C);
    float Vx_C = Target_Velocity_X + Chassis_Vr_C * sin(THETA_C);

    // 计算D轮的速度分量
    float Vy_D = Target_Velocity_Y - Chassis_Vr_D * cos(THETA_D);
    float Vx_D = Target_Velocity_X + Chassis_Vr_D * sin(THETA_D);
    for (int i = 0; i < 4; i++)
    {
        float Vx, Vy;
        switch (i)
        {
        case 0:
            Vx = Vx_A;
            Vy = Vy_A;
            break;
        case 1:
            Vx = Vx_B;
            Vy = Vy_B;
            break;
        case 2:
            Vx = Vx_C;
            Vy = Vy_C;
            break;
        case 3:
            Vx = Vx_D;
            Vy = Vy_D;
            break;
        }

        // 计算舵向角度，弧度制单位，范围-PI到PI
        Target_Steer_Angle_Rad[i] = My_atan(Vy, Vx);

        // 计算轮子线速度
        float linear_vel = sqrt(Vx * Vx + Vy * Vy);

        // 转换为轮子角速度
        Target_Wheel_Omega[i] = linear_vel * VEL2RPM;
    }

#endif
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
#ifdef steering_wheel
    // 舵向底盘坐标系转化为轮组坐标系
    AGV_DirectiveMotor_TargetStatus_To_MotorAngle_In_ChassisCoordinate();
#endif
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

void Class_Tricycle_Chassis::AGV_DirectiveMotor_TargetStatus_To_MotorAngle_In_ChassisCoordinate()
{
    //     //数据初始化；更新
    // 	chassis->A_motor.last_target_angle = chassis->A_motor.target_angle;
    // 	chassis->B_motor.last_target_angle = chassis->B_motor.target_angle;
    // 	chassis->C_motor.last_target_angle = chassis->C_motor.target_angle;
    // 	chassis->D_motor.last_target_angle = chassis->D_motor.target_angle;

    // 	//	chassis->A_motor.target_angle = chassis->A_motor.ChassisCoordinate_Angle + chassis->A_motor.zero_position;
    // 	//	chassis->B_motor.target_angle = chassis->B_motor.ChassisCoordinate_Angle + chassis->B_motor.zero_position;
    // 	//	chassis->C_motor.target_angle = chassis->C_motor.ChassisCoordinate_Angle + chassis->C_motor.zero_position;
    // 	//	chassis->D_motor.target_angle = chassis->D_motor.ChassisCoordinate_Angle + chassis->D_motor.zero_position;

    // 	// 轮子方向超前坐标方向90°
    // 	chassis->A_motor.target_angle = chassis->A_motor.ChassisCoordinate_Angle - PI / 2 / (2 * PI) * 8191;
    // 	chassis->B_motor.target_angle = chassis->B_motor.ChassisCoordinate_Angle - PI / 2 / (2 * PI) * 8191;
    // 	chassis->C_motor.target_angle = chassis->C_motor.ChassisCoordinate_Angle - PI / 2 / (2 * PI) * 8191;
    // 	chassis->D_motor.target_angle = chassis->D_motor.ChassisCoordinate_Angle - PI / 2 / (2 * PI) * 8191;
    //     //刹车模式
    // 	if (Chassis_Control_Type_DISABLE)
    // 	{
    // 		chassis->A_motor.target_angle = 8191 * (315) / 360;
    // 		chassis->B_motor.target_angle = 8191 * (225) / 360;
    // 		chassis->C_motor.target_angle = 8191 * (135) / 360;
    // 		chassis->D_motor.target_angle = 8191 * (45) / 360;
    // 	}
    //     //限幅
    // 	if (chassis->A_motor.target_angle > 8191)
    // 		chassis->A_motor.target_angle -= 8191;
    // 	if (chassis->A_motor.target_angle < 0)
    // 		chassis->A_motor.target_angle += 8191;
    // 	if (chassis->B_motor.target_angle > 8191)
    // 		chassis->B_motor.target_angle -= 8191;
    // 	if (chassis->B_motor.target_angle < 0)
    // 		chassis->B_motor.target_angle += 8191;
    // 	if (chassis->C_motor.target_angle > 8191)
    // 		chassis->C_motor.target_angle -= 8191;
    // 	if (chassis->C_motor.target_angle < 0)
    // 		chassis->C_motor.target_angle += 8191;
    // 	if (chassis->D_motor.target_angle > 8191)
    // 		chassis->D_motor.target_angle -= 8191;
    // 	if (chassis->D_motor.target_angle < 0)
    // 		chassis->D_motor.target_angle += 8191;
    // //计算速度输出值
    // 	chassis->A_motor.target_speed.output = (int)(chassis->A_motor.target_speed.rpm) * 1.0f;
    // 	chassis->B_motor.target_speed.output = (int)(chassis->B_motor.target_speed.rpm) * 1.0f;
    // 	chassis->C_motor.target_speed.output = (int)(chassis->C_motor.target_speed.rpm) * 1.0f;
    // 	chassis->D_motor.target_speed.output = (int)(chassis->D_motor.target_speed.rpm) * 1.0f;

    // 计算实际角度（考虑90度偏移）
    float actual_angle_A_rad = Target_Steer_Angle_Rad[0] - (PI / 2.0f);
    float actual_angle_B_rad = Target_Steer_Angle_Rad[1] - (PI / 2.0f);
    float actual_angle_C_rad = Target_Steer_Angle_Rad[2] - (PI / 2.0f);
    float actual_angle_D_rad = Target_Steer_Angle_Rad[3] - (PI / 2.0f);

    // 获取各个电机的速度大小
    float speed_A = -Target_Wheel_Omega[0];
    float speed_B = -Target_Wheel_Omega[1];
    float speed_C = -Target_Wheel_Omega[2];
    float speed_D = -Target_Wheel_Omega[3];

    // 如果是刹车模式
    if (Target_Velocity_X==0&&Target_Velocity_Y==0&&Target_Omega==0)
    {
        // 设置X形刹车角度
        actual_angle_A_rad = 315.0f * PI / 180.0f; // A轮
        actual_angle_B_rad = 225.0f * PI / 180.0f; // B轮
        actual_angle_C_rad = 135.0f * PI / 180.0f; // C轮
        actual_angle_D_rad = 45.0f * PI / 180.0f;  // D轮
    }
    // 速度限制
    int temp = 0;
    temp = Float_Abs(speed_A);
    if (Float_Abs(speed_B) > temp)
        temp = Float_Abs(speed_B);
    if (Float_Abs(speed_C) > temp)
        temp = Float_Abs(speed_C);
    if (Float_Abs(speed_D) > temp)
        temp = Float_Abs(speed_D);

    if (temp > MAX_MOTOR_SPEED)
    {
        speed_A = (int)(speed_A * MAX_MOTOR_SPEED * 1.0 / temp);
        speed_B = (int)(speed_B * MAX_MOTOR_SPEED * 1.0 / temp);
        speed_C = (int)(speed_C * MAX_MOTOR_SPEED * 1.0 / temp);
        speed_D = (int)(speed_D * MAX_MOTOR_SPEED * 1.0 / temp);
    }
    else
    {
        speed_A = (int)(speed_A);
        speed_B = (int)(speed_B);
        speed_C = (int)(speed_C);
        speed_D = (int)(speed_D);
    }
    // 计算A轮速度分量
    float vx_A = speed_A * cos(actual_angle_A_rad) *RPM2VEL;
    float vy_A = speed_A * sin(actual_angle_A_rad) *RPM2VEL;

    // 计算B轮速度分量
    float vx_B = speed_B * cos(actual_angle_B_rad) *RPM2VEL;
    float vy_B = speed_B * sin(actual_angle_B_rad) *RPM2VEL;

    // 计算C轮速度分量
    float vx_C = speed_C * cos(actual_angle_C_rad) *RPM2VEL;
    float vy_C = speed_C * sin(actual_angle_C_rad) *RPM2VEL;

    // 计算D轮速度分量
    float vx_D = speed_D * cos(actual_angle_D_rad) *RPM2VEL;
    float vy_D = speed_D * sin(actual_angle_D_rad) *RPM2VEL;

// 应用功率限制
#ifdef POWER_LIMIT
    float power_ratio = Power_Limit.Get_Power_Ratio();
    vx_A *= power_ratio;
    vy_A *= power_ratio;
    vx_B *= power_ratio;
    vy_B *= power_ratio;
    vx_C *= power_ratio;
    vy_C *= power_ratio;
    vx_D *= power_ratio;
    vy_D *= power_ratio;
#endif

    
    Agv_Board[0].Set_Target_Velocity_X(vx_A);
    Agv_Board[0].Set_Target_Velocity_Y(vy_A);

    Agv_Board[1].Set_Target_Velocity_X(vx_B);
    Agv_Board[1].Set_Target_Velocity_Y(vy_B);

    Agv_Board[2].Set_Target_Velocity_X(vx_C);
    Agv_Board[2].Set_Target_Velocity_Y(vy_C);

    Agv_Board[3].Set_Target_Velocity_X(vx_D);
    Agv_Board[3].Set_Target_Velocity_Y(vy_D);
    
    Agv_Board[0].Output();
    Agv_Board[1].Output();
    Agv_Board[2].Output();
    Agv_Board[3].Output();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
