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

// float Length15_K[12]={-47.8415   , 2.9752  , 16.1304  , 17.7654, -151.3596 ,  -9.7626,0.0595  , -0.0261 ,  -0.1262 ,  -0.1324   ,0.5629  ,  0.0416};
float Length15_K[12] = {-5.6837, -0.4468, -0.7518, -1.1037, 4.8769, 0.8958,
                        15.8922, 1.3973, 4.2834, 5.4264, 1.4743, -0.2298};
// float Length15_K[12]={ -7.6105  , -0.4364 ,  -0.1000 ,  -0.4426   , 3.4183 ,   0.6046,
//     3.3106   , 0.0532 ,  -0.0007 ,  -0.0078  , 11.7270  ,  1.6998};
//  float Legth25_K[12]={-16.0594,-1.2155,-0.6897,-1.3989,6.2852,0.5868,42.7543,3.2848,3.3125,6.4525,13.4492,-0.0461};
float Length25_K[12] = {-10.2319, -0.9229, -0.6210, -1.1780, 4.5019, 0.9968,
                        16.0409, 1.7810, 2.0531, 3.7178, 17.1216, 2.6194};
float Length30_K[12] = {-17.9076f, -1.4784f, -0.7372f, -1.4999f, 5.7243f, 0.5674f, 41.2367f, 3.3465f, 2.8742f, 5.6139f, 16.8290f, 0.2295};
float Length11_K[12] = {-5.2309, -0.3819, -1.0740, -1.3189, 8.1955, 1.2616,
                        9.4638, 0.7460, 3.6165, 4.0450, 7.6335, 0.3966};

void Class_Tricycle_Chassis::mySaturate(float *in, float min, float max)
{
    if (*in < min)
    {
        *in = min;
    }
    else if (*in > max)
    {
        *in = max;
    }
}

void Class_Tricycle_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max, float __Steer_Power_Ratio)
{
    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
    Steer_Power_Ratio = __Steer_Power_Ratio;
#ifdef SPEED_SLOPE
    // 斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.004f, 0.008f);
    // 斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.004f, 0.008f);
    // 斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);
#endif
#ifdef SUPERCAP
    // 超级电容初始化
    Supercap.Init(&hcan2, 45);
#endif
#ifdef C_IMU
    Boardc_BMI.Init();

#endif

    // 电机PID批量初始化
    Yaw_Motor.Init(&hcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, DJI_Motor_Control_Mode_CURRENT, 0);
    Yaw_Motor.PID_Angle.Init(1.0f, 0.0f, 0.0f, 0.0f, Yaw_Motor.Get_Output_Max(), Yaw_Motor.Get_Output_Max());
    Yaw_Motor.PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, Yaw_Motor.Get_Output_Max(), Yaw_Motor.Get_Output_Max());

    Joint_Motor[0].motor.Init(&hcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.f, 15.f); // 6.16621923
    Joint_Motor[1].motor.Init(&hcan1, DM_Motor_ID_0xA2, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.f, 15.f); // 6.34684563
    Joint_Motor[2].motor.Init(&hcan2, DM_Motor_ID_0xA3, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.f, 15.f); // 6.42469501
    Joint_Motor[3].motor.Init(&hcan2, DM_Motor_ID_0xA4, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.f, 15.f); // 6.29660797

    //	Joint_Motor[0].motor.Init(&hcan1, DM_Motor_ID_0xA1,DM_Motor_Control_Method_POSITION_OMEGA,0,1.f,35.f);
    //    Joint_Motor[1].motor.Init(&hcan1, DM_Motor_ID_0xA2,DM_Motor_Control_Method_POSITION_OMEGA,0,1.f,35.f);
    //    Joint_Motor[2].motor.Init(&hcan1, DM_Motor_ID_0xA3,DM_Motor_Control_Method_POSITION_OMEGA,0,1.f,35.f);
    //    Joint_Motor[3].motor.Init(&hcan1, DM_Motor_ID_0xA4,DM_Motor_Control_Method_POSITION_OMEGA,0,1.f,35.f);

    Wheel_Motor[0].motor.Init(&hcan1, AK_Motor_ID_0x01, CAN_PACKET_SET_CURRENT, true);
    Wheel_Motor[1].motor.Init(&hcan2, AK_Motor_ID_0x02, CAN_PACKET_SET_CURRENT, false);
    leg_set = 0.28;
    Left_Leg.VMC_Init();
    Left_Leg.Set_LQR_K(Length15_K);
    Right_Leg.VMC_Init();
    Right_Leg.Set_LQR_K(Length15_K);

    left_leg_length_pid.Init(1200.0f, 0.0f, 3.0f, 0.0f, 0.0f, 150.f);
    right_leg_length_pid.Init(1200.0f, 0.0f, 3.0f, 0.0f, 0.0f, 150.f);
    roll_pid.Init(140.0f, 0.0f, 10.0f, 0.0f, 0.0f, 1.0f);
    Tp_pid.Init(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f);
    turn_pid.Init(0.06f, 0.0f, 0.005f, 0.0f, 0.0f, 1.0f);

    test_left_leg_theta_pid.Init(120.0f, 0.0f, 0.7f, 0.0f, 0.0f, 10.0f);
    test_right_leg_theta_pid.Init(120.0f, 0.0f, 0.7f, 0.0f, 0.0f, 10.0f);

    /*Observe*/
    Observe.Boardc_BMI = &Boardc_BMI;
    Observe.Left_Leg = &Left_Leg;
    Observe.Right_Leg = &Right_Leg;
    Observe.Joint_Motor[0] = &Joint_Motor[0];
    Observe.Joint_Motor[1] = &Joint_Motor[1];
    Observe.Joint_Motor[2] = &Joint_Motor[2];
    Observe.Joint_Motor[3] = &Joint_Motor[3];
    Observe.Wheel_Motor[0] = &Wheel_Motor[0];
    Observe.Wheel_Motor[1] = &Wheel_Motor[1];
    Observe.xvEstimateKF_Init();

    // for (int i = 0; i < 4; i++)
    // {
    //     Motor_Wheel[i].PID_Omega.Init(1500.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    // }

    // //轮向电机ID初始化
    // Motor_Wheel[0].Init(&hcan1, DJI_Motor_ID_0x201);
    // Motor_Wheel[1].Init(&hcan1, DJI_Motor_ID_0x202);
    // Motor_Wheel[2].Init(&hcan1, DJI_Motor_ID_0x203);
    // Motor_Wheel[3].Init(&hcan1, DJI_Motor_ID_0x204);
}

// void Class_Tricycle_Chassis::Pose_Calculate(void)
// {

//     // if(Boardc_BMI.Get_Rad_Pitch()<(3.1415926f/6.0f)&&Boardc_BMI.Get_Rad_Pitch()>(-3.1415926f/6.0f))
//     // {//根据pitch角度判断倒地自起是否完成
//     // recover_flag=0;
//     // }

//     Left_Leg.phi1 = Joint_Motor[1].motor.Get_Now_Angle() - 7.653f + PI;
//     Left_Leg.phi4 = Joint_Motor[0].motor.Get_Now_Angle() - 4.76f;

//     Right_Leg.phi1 = Joint_Motor[2].motor.Get_Now_Angle() - 7.7f + PI;
//     Right_Leg.phi4 = Joint_Motor[3].motor.Get_Now_Angle() - 4.865f;

//     mypitch = Boardc_BMI.Get_Rad_Pitch();
//     mypitchgyro = Boardc_BMI.Get_Gyro_Pitch();

//     total_yaw = Boardc_BMI.Get_True_Angle_Total_Yaw();
//     roll = Boardc_BMI.Get_Rad_Roll();
//     leg_theta_err = (Left_Leg.theta - Right_Leg.theta);
//     // if(Boardc_BMI.Get_Rad_Pitch()<(PI/6.0f)&&Boardc_BMI.Get_Rad_Pitch()>(-PI/6.0f))
//     // recover_flag=0;
// }
/**
 * @brief 速度解算
 *
 */
float torque_limition = 30.0f, torque_limition_wheel = 0;
float test;
float temp_test_1, temp_test_2, temp_test_3, temp_test_4;
float tmp_wheel_t[2];

void Class_Tricycle_Chassis::Speed_Resolution()
{

    CHASSIS_DWT_Dt = DWT_GetDeltaT(&CHASSIS_DWT_Count);

    Left_Leg.VMC_calc_1(CHASSIS_DWT_Dt);
    Right_Leg.VMC_calc_1(CHASSIS_DWT_Dt);
    x_filter = Observe.Get_X_Filter();
    v_filter = Observe.Get_V_Filter();

     turn_pid.Set_Now(total_yaw);
     turn_pid.Set_Target(yaw_set);
     turn_pid.TIM_Adjust_PeriodElapsedCallback();
     turn_T=turn_pid.Get_Out();

    // roll_pid.Set_Now(Roll.Get_True_Gyro_Roll());
    // roll_pid.Set_Target(roll_set-roll);
    // roll_pid.TIM_Adjust_PeriodElapsedCallback();
    // roll_f0=roll_pid.Get_Out();

    //  Tp_pid.Set_Now(leg_theta_err);
    //  Tp_pid.Set_Target(0.0f);
    //  Tp_pid.TIM_Adjust_PeriodElapsedCallback();
    //  leg_tp=Tp_pid.Get_Out();

    // tmp_wheel_t[0] = (Left_Leg.LQR_K[0] * (Left_Leg.theta) + Left_Leg.LQR_K[1] * (Left_Leg.d_theta) + Left_Leg.LQR_K[2] * (x_set - x_filter) + Left_Leg.LQR_K[3] * (v_set - v_filter) + Left_Leg.LQR_K[4] * (mypitch) + Left_Leg.LQR_K[5] * (mypitchgyro)); // mypitch,mypitchgyro

    // tmp_wheel_t[1] = (Right_Leg.LQR_K[0] * (Right_Leg.theta) + Right_Leg.LQR_K[1] * (Right_Leg.d_theta) + Right_Leg.LQR_K[2] * (x_set - x_filter) + Right_Leg.LQR_K[3] * (v_set - v_filter) + Right_Leg.LQR_K[4] * (mypitch) + Right_Leg.LQR_K[5] * (mypitchgyro)); // mypitch,mypitchgyro

    // Left_Leg.Tp = (Left_Leg.LQR_K[6] * (Left_Leg.theta) + Left_Leg.LQR_K[7] * (Left_Leg.d_theta) + Left_Leg.LQR_K[8] * (x_set - x_filter) + Left_Leg.LQR_K[9] * (v_set - v_filter) + Left_Leg.LQR_K[10] * (mypitch) + Left_Leg.LQR_K[11] * (mypitchgyro));
    // //

    // Left_Leg.Tp = Left_Leg.Tp - leg_tp; // 髋关节输出力矩
    // //		Left_Leg.Tp=test;
    // Right_Leg.Tp = (Right_Leg.LQR_K[6] * (Right_Leg.theta) + Right_Leg.LQR_K[7] * (Right_Leg.d_theta) + Right_Leg.LQR_K[8] * (x_set - x_filter) + Right_Leg.LQR_K[9] * (v_set - v_filter) + Right_Leg.LQR_K[10] * (mypitch) + Right_Leg.LQR_K[11] * (mypitchgyro));

    Right_Leg.Tp = Right_Leg.Tp + leg_tp; // 髋关节输出力矩
    //	Right_Leg.Tp=test;
    Wheel_Motor[0].wheel_T = -((tmp_wheel_t[0] - turn_T) / 6.f / 0.091f); // 轮毂电机输出力矩
    Wheel_Motor[1].wheel_T = -((tmp_wheel_t[1] + turn_T) / 6.f / 0.091f); // 轮毂电机输出力矩
    mySaturate(&Wheel_Motor[0].wheel_T, -torque_limition_wheel, torque_limition_wheel);
    mySaturate(&Wheel_Motor[1].wheel_T, -torque_limition_wheel, torque_limition_wheel);

    right_leg_length_pid.Set_Now(Right_Leg.L0);
    right_leg_length_pid.Set_Target(leg_set);
    right_leg_length_pid.TIM_Adjust_PeriodElapsedCallback();
    Right_Leg.F0 = total_mg / 2.f * arm_cos_f32(Right_Leg.theta) + right_leg_length_pid.Get_Out(); // 前馈+pd
    temp_test_2 = arm_cos_f32(Right_Leg.theta);

    left_leg_length_pid.Set_Now(Left_Leg.L0);
    left_leg_length_pid.Set_Target(leg_set);
    left_leg_length_pid.TIM_Adjust_PeriodElapsedCallback();
    Left_Leg.F0 = total_mg / 2.f * arm_cos_f32(Left_Leg.theta) + left_leg_length_pid.Get_Out(); // 前馈+pd

    Left_Leg.VMC_calc_2();
    Right_Leg.VMC_calc_2();

    mySaturate(&Left_Leg.torque_set[1], -torque_limition, torque_limition);
    mySaturate(&Left_Leg.torque_set[0], -torque_limition, torque_limition);
    mySaturate(&Right_Leg.torque_set[1], -torque_limition, torque_limition);
    mySaturate(&Right_Leg.torque_set[0], -torque_limition, torque_limition);
}

/*重写                             */
void Class_Tricycle_Chassis::chassisL_feedback_update()
{
    Left_Leg.phi1 = PI / 2.0f + Joint_Motor[2].motor.Get_Now_Angle();
    Left_Leg.phi4 = PI / 2.0f + Joint_Motor[3].motor.Get_Now_Angle();

    myPithL = pitch_offset*DEG_TO_RAD + 0.00f - Boardc_BMI.Get_Rad_Pitch();
    myPithGyroL = 0.0f - Boardc_BMI.Get_Gyro_Pitch();
}

void Class_Tricycle_Chassis::chassisR_feedback_update()
{
    Right_Leg.phi1 = PI / 2.0f + Joint_Motor[0].motor.Get_Now_Angle();
    Right_Leg.phi4 = PI / 2.0f + Joint_Motor[1].motor.Get_Now_Angle();

    myPithR = -pitch_offset*DEG_TO_RAD+Boardc_BMI.Get_Rad_Pitch();
    myPithGyroR = Boardc_BMI.Get_Gyro_Pitch();

    turn_pid.Set_Now(total_yaw);
    turn_pid.Set_Target(yaw_set);
    turn_pid.TIM_Adjust_PeriodElapsedCallback();
    turn_T = turn_pid.Get_Out();
    total_yaw =- Boardc_BMI.QEKF_INS.YawTotalAngle;

    roll = Boardc_BMI.Get_Angle_Roll();
    leg_theta_err = 0.0f - (Right_Leg.theta + Left_Leg.theta);

    if (Boardc_BMI.Get_Angle_Pitch() < ((3.1415926f / 6.0f)) && Boardc_BMI.Get_Angle_Pitch() > (-3.1415926f / 6.0f))
    {
        // 根据pitch角度判断倒地自起是否完成
        recover_flag = 0;
    }
}
float test_angle = PI / 2.0f;
void Class_Tricycle_Chassis::chassisL_control_loop()
{

    static uint32_t chassisL_control_loop_cnt = 0;
    static float chassisL_control_loop_dt = 0.0;

    chassisL_control_loop_dt = DWT_GetDeltaT(&chassisL_control_loop_cnt);

    float temp_T = 0;  // 轮毂电机输出力矩
    float temp_Tp = 0; // 髋关节输出力矩

    Left_Leg.Set_Pitch(myPithL);
    Left_Leg.Set_Pitch_Gyro(myPithGyroL);

    Left_Leg.VMC_calc_1(chassisL_control_loop_dt);

    // 得到LQR矩阵系数
    for (int i = 0; i < 12; i++)
    {
        Left_Leg.LQR_K[i] = Left_Leg.LQR_K_calc(&Poly_Coefficient[i][0], Left_Leg.L0);
    }

    // 根据系数计算输出量U（T，Tp）
    // 注意这里x和v的方向
    temp_T = T_coefficient * (Left_Leg.LQR_K[0] * (Left_Leg.theta) + Left_Leg.LQR_K[1] * (Left_Leg.d_theta) + Left_Leg.LQR_K[2] * (x_set - x_filter) + Left_Leg.LQR_K[3] * (v_set - v_filter) + Left_Leg.LQR_K[4] * (myPithL) + Left_Leg.LQR_K[5] * (myPithGyroL));

    Left_Leg.tp_x_out = (x_set - x_filter) * Left_Leg.LQR_K[8];
    Left_Leg.tp_v_out = (v_set - v_filter) * Left_Leg.LQR_K[9];
    Left_Leg.tp_myPith_out = myPithL * Left_Leg.LQR_K[10];
    Left_Leg.tp_myPithGyro_out = myPithGyroL * Left_Leg.LQR_K[11];
    Left_Leg.tp_theta_out = Left_Leg.theta * Left_Leg.LQR_K[6];
    Left_Leg.tp_dtheta_out = Left_Leg.d_theta * Left_Leg.LQR_K[7];

    Left_Leg.t_x_out = Left_Leg.LQR_K[2] * (x_set - x_filter);
    Left_Leg.t_v_out = Left_Leg.LQR_K[3] * (v_set - v_filter);
    Left_Leg.t_myPith_out = Left_Leg.LQR_K[4] * (myPithL);
    Left_Leg.t_myPithGyro_out = Left_Leg.LQR_K[5] * (myPithGyroL);
    Left_Leg.t_theta_out = Left_Leg.LQR_K[0] * (Left_Leg.theta);
    Left_Leg.t_dtheta_out = Left_Leg.LQR_K[1] * (Left_Leg.d_theta);

    temp_Tp = T_coefficient*(Left_Leg.LQR_K[6] * (Left_Leg.theta) + Left_Leg.LQR_K[7] * (Left_Leg.d_theta) + Left_Leg.LQR_K[8] * (x_set - x_filter) + Left_Leg.LQR_K[9] * (v_set - v_filter) + Left_Leg.LQR_K[10] * (myPithL) + Left_Leg.LQR_K[11] * (myPithGyroL));

    Wheel_Motor[1].wheel_T = temp_T + turn_T;
    // mySaturate(&Wheel_Motor[1].wheel_T, -1.0f, 1.0f);

    Left_Leg.Tp = 1.0f * temp_Tp + leg_tp;

    //Left_Leg.Tp = 0;

    left_leg_length_pid.Set_Now(Left_Leg.L0);
    left_leg_length_pid.Set_Target(leg_set);
    left_leg_length_pid.TIM_Adjust_PeriodElapsedCallback();
    Left_Leg.F0 = total_mg / 2.f /* * arm_cos_f32(Left_Leg.theta)*/ + left_leg_length_pid.Get_Out(); //

    test_left_leg_theta_pid.Set_Now(Left_Leg.phi0);
    test_left_leg_theta_pid.Set_Target(test_angle);
    test_left_leg_theta_pid.TIM_Adjust_PeriodElapsedCallback();

    //Left_Leg.Tp = test_left_leg_theta_pid.Get_Out();

    // Left_Leg.F0 = 0;
    /*******************************/
    Left_Leg.FN = Left_Leg.F0 * arm_cos_f32(Left_Leg.theta) + Left_Leg.Tp * arm_sin_f32(Left_Leg.theta) / Left_Leg.L0 + wheel_mg; // 腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度

    /*******************************/

    mySaturate(&Left_Leg.torque_set[1], -torque_limition, torque_limition);
    mySaturate(&Left_Leg.torque_set[0], -torque_limition, torque_limition);
    mySaturate(&Left_Leg.Tp, -torque_limition, torque_limition);

    Left_Leg.VMC_calc_2(); // 计算期望的关节输出力矩

    Joint_Motor[2].motor.Set_Target_Torque(Left_Leg.torque_set[0]);
    Joint_Motor[3].motor.Set_Target_Torque(Left_Leg.torque_set[1]);
    Wheel_Motor[1].motor.Set_Target_Current(Wheel_Motor[1].wheel_T / Wheel_Motor[1].motor.Get_KT());
}

void Class_Tricycle_Chassis::chassisR_control_loop()
{
    static uint32_t chassisR_control_loop_cnt = 0;
    static float chassisR_control_loop_dt = 0.0;

    chassisR_control_loop_dt = DWT_GetDeltaT(&chassisR_control_loop_cnt);

    float temp_T = 0;  // 轮毂电机输出力矩
    float temp_Tp = 0; // 髋关节输出力矩

    Right_Leg.Set_Pitch(myPithR);
    Right_Leg.Set_Pitch_Gyro(myPithGyroR);

    Right_Leg.VMC_calc_1(chassisR_control_loop_dt);

    // 得到LQR矩阵系数
    for (int i = 0; i < 12; i++)
    {
        Right_Leg.LQR_K[i] = Right_Leg.LQR_K_calc(&Poly_Coefficient[i][0], Right_Leg.L0);
    }
    // 根据系数计算输出量U（T，Tp）

    // 根据系数计算输出量U（T，Tp）
    temp_T = T_coefficient * (Right_Leg.LQR_K[0] * (Right_Leg.theta) + Right_Leg.LQR_K[1] * (Right_Leg.d_theta) + Right_Leg.LQR_K[2] * (x_filter - x_set) + Right_Leg.LQR_K[3] * (v_filter - v_set) + Right_Leg.LQR_K[4] * (myPithR) + Right_Leg.LQR_K[5] * (myPithGyroR));

    Right_Leg.tp_theta_out = Right_Leg.theta * Right_Leg.LQR_K[6];
    Right_Leg.tp_dtheta_out = Right_Leg.d_theta * Right_Leg.LQR_K[7];
    Right_Leg.tp_x_out = (x_filter - x_set) * Right_Leg.LQR_K[8];
    Right_Leg.tp_v_out = (v_filter - v_set) * Right_Leg.LQR_K[9];
    Right_Leg.tp_myPith_out = myPithR * Right_Leg.LQR_K[10];
    Right_Leg.tp_myPithGyro_out = myPithGyroR * Right_Leg.LQR_K[11];

    Right_Leg.t_x_out = Right_Leg.LQR_K[2] * (x_filter - x_set);
    Right_Leg.t_v_out = Right_Leg.LQR_K[3] * (v_filter - v_set);
    Right_Leg.t_myPith_out = Right_Leg.LQR_K[4] * (myPithR);
    Right_Leg.t_myPithGyro_out = Right_Leg.LQR_K[5] * (myPithGyroR);
    Right_Leg.t_theta_out = Right_Leg.LQR_K[0] * (Right_Leg.theta);
    Right_Leg.t_dtheta_out = Right_Leg.LQR_K[1] * (Right_Leg.d_theta);

    temp_Tp = T_coefficient*(Right_Leg.LQR_K[6] * (Right_Leg.theta) + Right_Leg.LQR_K[7] * (Right_Leg.d_theta) + Right_Leg.LQR_K[8] * (x_filter - x_set) + Right_Leg.LQR_K[9] * (v_filter - v_set) + Right_Leg.LQR_K[10] * (myPithR) + Right_Leg.LQR_K[11] * (myPithGyroR));

    Wheel_Motor[0].wheel_T = temp_T + turn_T;
    // mySaturate(&Wheel_Motor[0].wheel_T, -1.0f, 1.0f);

    Right_Leg.Tp = 1.0f * temp_Tp + leg_tp;

   // Right_Leg.Tp = 0;

    right_leg_length_pid.Set_Now(Right_Leg.L0);
    right_leg_length_pid.Set_Target(leg_set);
    right_leg_length_pid.TIM_Adjust_PeriodElapsedCallback();

    Right_Leg.F0 = total_mg / 2.f /** arm_cos_f32(Right_Leg.theta) */ + right_leg_length_pid.Get_Out(); //

    // Right_Leg.F0 = 0;

    test_right_leg_theta_pid.Set_Now(Right_Leg.phi0);
    test_right_leg_theta_pid.Set_Target(test_angle);
    test_right_leg_theta_pid.TIM_Adjust_PeriodElapsedCallback();

   // Right_Leg.Tp = test_right_leg_theta_pid.Get_Out();
    /*******************************/
    Right_Leg.FN = Right_Leg.F0 * arm_cos_f32(Right_Leg.theta) + Right_Leg.Tp * arm_sin_f32(Right_Leg.theta) / Right_Leg.L0 + wheel_mg; // 腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度

    /*******************************/

    mySaturate(&Right_Leg.torque_set[1], -torque_limition, torque_limition);
    mySaturate(&Right_Leg.torque_set[0], -torque_limition, torque_limition);
    mySaturate(&Right_Leg.Tp, -torque_limition, torque_limition);

    Right_Leg.VMC_calc_2(); // 计算期望的关节输出力矩

    Joint_Motor[0].motor.Set_Target_Torque(Right_Leg.torque_set[0]);
    Joint_Motor[1].motor.Set_Target_Torque(Right_Leg.torque_set[1]);
    Wheel_Motor[0].motor.Set_Target_Current(Wheel_Motor[0].wheel_T / Wheel_Motor[0].motor.Get_KT());
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float Power_Limit_K = 1.0f;
uint64_t delta_time = 0;
static uint64_t start_time = 0;
void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
#ifdef C_IMU

#endif
    start_time = DWT_GetTimeline_us();
    static uint8_t mod5 = 0;

    Yaw_Motor.Set_Target_Angle(test);
    Yaw_Motor.TIM_PID_PeriodElapsedCallback();

    CHASSIS_DWT_Dt = DWT_GetDeltaT(&CHASSIS_DWT_Count); // s

    Observe.TIM_Calculate_PeriodElapsedCallback(CHASSIS_DWT_Dt);
    x_filter = Observe.x_filter;
    v_filter = Observe.v_filter;

    chassisL_feedback_update();
    chassisR_feedback_update();

    if (mod5 == 1 || mod5 == 3)
    {
        chassisL_control_loop();
        chassisR_control_loop();
    }

    mod5++;
    if (mod5 == 5)
    {
        mod5 = 0;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        Joint_Motor[i].motor.Set_Target_Torque(0.0);
        Joint_Motor[i]
            .motor.TIM_Process_PeriodElapsedCallback();
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        Wheel_Motor[i].motor.Set_Target_Current(0.0f);
        Wheel_Motor[i].motor.Task_Process_PeriodElapsedCallback();
    }
    delta_time = DWT_GetTimeline_us() - start_time;
#ifdef SUPERCAP
/****************************超级电容***********************************/
#ifdef REFEREE
    Supercap.Set_Now_Power(Referee->Get_Chassis_Power());

    if (Referee->Get_Referee_Status() == Referee_Status_DISABLE)
        Supercap.Set_Limit_Power(45.0f);
    else
    {
        float offset;
        offset = (Referee->Get_Chassis_Energy_Buffer() - 20.0f) / 4;
        Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() + offset);
    }
#else
    Supercap.Set_Limit_Power(45.0f);
#endif
    Supercap.TIM_Supercap_PeriodElapsedCallback();
#endif
/*************************功率限制策略*******************************/
#ifdef POWER_LIMIT
    if (__Sprint_Status == Sprint_Status_ENABLE)
    {
// 功率限制
#ifdef REFEREE
        Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max() * 1.5f);
#else
        Power_Limit.Set_Power_Limit(45.0f);
#endif
    }
    else
    {
#ifdef REFEREE
        Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max());
#else
        Power_Limit.Set_Power_Limit(30.0f);
#endif
    }

// Power_Limit.Set_Motor(Motor_Wheel);   //添加四个电机的控制电流和当前转速
#ifdef REFEREE
    Power_Limit.Set_Chassis_Buffer(Referee->Get_Chassis_Energy_Buffer());
#else
    Power_Limit.Set_Chassis_Buffer(30);
#endif

    if (Supercap.Get_Supercap_Status() == Supercap_Status_DISABLE)
        Power_Limit.Set_Supercap_Enegry(0.0f);
    else
        Power_Limit.Set_Supercap_Enegry(Supercap.Get_Stored_Energy());

// Power_Limit.TIM_Adjust_PeriodElapsedCallback(Motor_Wheel);  //功率限制算法
#endif
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
