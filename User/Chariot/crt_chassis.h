/**
 * @file crt_chassis.h
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

#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

/* Includes ------------------------------------------------------------------*/

#include "alg_slope.h"
#include "dvc_sampler.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"
#include "alg_power_limit.h"
#include "dvc_supercap.h"
#include "config.h"
#include "dvc_imu.h"
#include "alg_vmc.h"
#include "dvc_dmmotor.h"
#include "dvc_AKmotor.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘冲刺状态枚举
 *
 */
enum Enum_Sprint_Status : uint8_t
{
    Sprint_Status_DISABLE = 0, 
    Sprint_Status_ENABLE,
};


/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type :uint8_t
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_FLLOW,
    Chassis_Control_Type_SPIN,
};

class Class_Chassis_Yaw
{
public:
    Class_IMU* IMU;
    
    inline float Get_True_Rad_Yaw();
    inline float Get_True_Gyro_Yaw();
    inline float Get_True_Angle_Yaw();
    inline float Get_True_Angle_Total_Yaw();

    void Transform_Angle();

protected:
    float True_Rad_Yaw = 0.0f;
    float True_Angle_Yaw = 0.0f;
    float True_Gyro_Yaw = 0.0f;
    float True_Angle_Total_Yaw = 0.0f;
};

class Class_Chassis_Pitch
{
public:
    Class_IMU* IMU;

    inline float Get_True_Rad_Pitch();
    inline float Get_True_Gyro_Pitch();
    inline float Get_True_Angle_Pitch();
    inline float Get_True_Angle_Total_Pitch();

    void Transform_Angle();

protected:
    float True_Rad_Pitch = 0.0f;
    float True_Angle_Pitch = 0.0f;
    float True_Gyro_Pitch = 0.0f;
    float True_Angle_Total_Pitch = 0.0f;
};

class Class_Chassis_Roll
{
public:
    Class_IMU* IMU;

    inline float Get_True_Rad_Roll();
    inline float Get_True_Gyro_Roll();
    inline float Get_True_Angle_Roll();
    inline float Get_True_Angle_Total_Roll();

    void Transform_Angle();

protected:
    float True_Rad_Roll = 0.0f;
    float True_Angle_Roll = 0.0f;
    float True_Gyro_Roll = 0.0f;
    float True_Angle_Total_Roll = 0.0f;
};

float Class_Chassis_Yaw::Get_True_Rad_Yaw()
{
    return True_Rad_Yaw;
}

float Class_Chassis_Yaw::Get_True_Gyro_Yaw()
{
    return True_Gyro_Yaw;
}

float Class_Chassis_Yaw::Get_True_Angle_Yaw()
{
    return True_Angle_Yaw;

}

float Class_Chassis_Yaw::Get_True_Angle_Total_Yaw()
{
    return True_Angle_Total_Yaw;
}

float Class_Chassis_Pitch::Get_True_Rad_Pitch()
{
    return True_Rad_Pitch;
}

float Class_Chassis_Pitch::Get_True_Gyro_Pitch()
{
    return True_Gyro_Pitch;

}
float Class_Chassis_Pitch::Get_True_Angle_Pitch()
{
    return True_Angle_Pitch;
}

float Class_Chassis_Pitch::Get_True_Angle_Total_Pitch()
{
    return True_Angle_Total_Pitch;
}

float Class_Chassis_Roll::Get_True_Rad_Roll()
{
    return True_Rad_Roll;

}

float Class_Chassis_Roll::Get_True_Gyro_Roll()
{
    return True_Gyro_Roll;
}

float Class_Chassis_Roll::Get_True_Angle_Roll()
{
    return True_Angle_Roll;
}

float Class_Chassis_Roll::Get_True_Angle_Total_Roll()
{
    return True_Angle_Total_Roll;

}

class Class_Joint_Motor
{
public:
   Class_DM_Motor_J4310 motor;
    
};

class Class_Wheel_Motor
{
public:
   Class_AK_Motor_80_6 motor;
   float wheel_T;
    
};

/**
 * @brief Specialized, 三轮舵轮底盘类
 *
 */
//omnidirectional 全向轮
class Class_Tricycle_Chassis
{
public:
    #ifdef SPEED_SLOPE
    //斜坡函数加减速速度X
    Class_Slope Slope_Velocity_X;
    //斜坡函数加减速速度Y
    Class_Slope Slope_Velocity_Y;
    //斜坡函数加减速角速度
    Class_Slope Slope_Omega;
    #endif
    #ifdef SUPERCAP
    Class_Supercap Supercap;
    #endif
    #ifdef C_IMU
    Class_IMU Boardc_BMI;
    Class_Chassis_Yaw Yaw;
    Class_Chassis_Pitch Pitch;
    Class_Chassis_Roll Roll;
    #endif
    #ifdef POWER_LIMIT
    //功率限制
    Class_Power_Limit Power_Limit;
    #endif
    
    #ifdef REFEREE
    //裁判系统
    Class_Referee *Referee;
    #endif
    //下方转动电机
    void mySaturate(float *in,float min,float max);
    void Pose_Calculate(void);
    float CHASSIS_DWT_Dt;
    uint32_t CHASSIS_DWT_Count;
    float mg=161.75f;
    Class_VMC Left_Leg;
    Class_VMC Right_Leg;

    Class_Joint_Motor Joint_Motor[4];
    Class_Wheel_Motor Wheel_Motor[2];


    Class_PID left_leg_length_pid;
    Class_PID right_leg_length_pid;
    Class_PID roll_pid;
    Class_PID Tp_pid;
    Class_PID turn_pid;

    float v_set;
    float x_set;
    float v_ramp_set;

    float yaw_set;
    float roll_set;
    float roll_x;
    float phi_set;
    float theta_set;

    float leg_set;
    float last_leg_set;

    float v_filter;
    float x_filter;

    float mypitch[2];
    float mypitchgyro[2];

    float roll;
    float total_yaw;
    float leg_theta_err;

    float turn_T;
    float roll_f0;

    float leg_tp;

    uint8_t start_flag;

    uint8_t prejump_flag;
    uint8_t recover_flag;

    uint8_t jump_flag[2];
    float jump_time[2];

    uint8_t suspend_flag[2];

    float Poly_Coefficient[12][4]={{-127.1708,186.3127,-110.7809,1.9832},
{3.5394,-0.8251,-5.6576,0.1987},
{-15.6548,17.8019,-7.1543,0.2274},
{-29.8907,34.1142,-13.9360,0.4134},
{61.6171,-45.0353,-0.8936,8.3963},
{7.2060,-6.8798,1.7269,0.4753},
{1164.0926,-1145.2650,329.9446,14.0979},
{70.2614,-77.6299,27.8075,0.1040},
{10.5235,5.8244,-14.6583,6.4689},
{22.3402,8.7217,-27.2194,12.4026},
{700.9154,-848.2240,374.2772,-37.9526},
{30.9163,-42.3907,21.8568,-3.3445}
};

    // Class_DJI_Motor_C620 Motor_Wheel[4];

    void Init(float __Velocity_X_Max = 4.0f, float __Velocity_Y_Max = 4.0f, float __Omega_Max = 8.0f, float __Steer_Power_Ratio = 0.5);

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline float Get_Velocity_X_Max();
    inline float Get_Velocity_Y_Max();
    inline float Get_Omega_Max();
    inline float Get_Now_Power();
    inline float Get_Now_Steer_Power();
    inline float Get_Target_Steer_Power();
    inline float Get_Now_Wheel_Power();
    inline float Get_Target_Wheel_Power();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();
    inline float Get_Spin_Omega();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Now_Velocity_X(float __Now_Velocity_X);
    inline void Set_Now_Velocity_Y(float __Now_Velocity_Y);
    inline void Set_Now_Omega(float __Now_Omega);

    inline void Set_Velocity_Y_Max(float __Velocity_Y_Max);
    inline void Set_Velocity_X_Max(float __Velocity_X_Max);

    void TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status);

protected:
    //初始化相关常量

    //速度X限制
    float Velocity_X_Max;
    //速度Y限制
    float Velocity_Y_Max;
    //角速度限制
    float Omega_Max;
    //舵向电机功率上限比率
    float Steer_Power_Ratio = 0.5f;
    //底盘小陀螺模式角速度
    float Spin_Omega = 4.0f;
    //常量

    //电机理论上最大输出
    float Steer_Max_Output = 30000.0f;
    float Wheel_Max_Output = 16384.0f;

    //内部变量

    //舵向电机目标值
    float Target_Steer_Angle[3];
    //转动电机目标值
    float Target_Wheel_Omega[4];

    //读变量

    //当前总功率
    float Now_Power = 0.0f;
    //当前舵向电机功率
    float Now_Steer_Power = 0.0f;
    //可使用的舵向电机功率
    float Target_Steer_Power = 0.0f;
    //当前轮向电机功率
    float Now_Wheel_Power = 0.0f;
    //可使用的轮向电机功率
    float Target_Wheel_Power = 0.0f;

    //写变量

    //读写变量

    //底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    //目标速度X
    float Target_Velocity_X = 0.0f;
    //目标速度Y
    float Target_Velocity_Y = 0.0f;
    //目标角速度
    float Target_Omega = 0.0f;
    //当前速度X
    float Now_Velocity_X = 0.0f;
    //当前速度Y
    float Now_Velocity_Y = 0.0f;
    //当前角速度
    float Now_Omega = 0.0f;

    //内部函数
    void Speed_Resolution();
};

/* Exported variables --------------------------------------------------------*/

//三轮车底盘参数

//轮组半径
const float WHEEL_RADIUS = 0.0520f;

//轮距中心长度
const float WHEEL_TO_CORE_DISTANCE[3] = {0.23724f, 0.21224f, 0.21224f};

//前心距中心长度
const float FRONT_CENTER_TO_CORE_DISTANCE = 0.11862f;

//前后轮距
const float FRONT_TO_REAR_DISTANCE = WHEEL_TO_CORE_DISTANCE[0] + FRONT_CENTER_TO_CORE_DISTANCE;

//前轮距前心
const float FRONT_TO_FRONT_CENTER_DISTANCE = 0.176f;

//轮组方位角
const float WHEEL_AZIMUTH[3] = {0.0f, atan2f(-FRONT_TO_FRONT_CENTER_DISTANCE, -FRONT_CENTER_TO_CORE_DISTANCE), atan2f(FRONT_TO_FRONT_CENTER_DISTANCE, -FRONT_CENTER_TO_CORE_DISTANCE)};

//轮子直径 单位m
const float WHELL_DIAMETER = 0.13f;	

//底盘半宽 单位m
const float HALF_WIDTH = 0.15f;		

//底盘半长 单位m
const float HALF_LENGTH = 0.15f;	

//转速转角速度	1 rpm = 2pi/60 rad/s 
const float RPM2RAD = 0.104720f;				

//转速转线速度	vel = rpn*pi*D/60  cm/s
const float RPM2VEL = 0.806342f;			

//线速度转转度  //1.240168							
const float VEL2RPM = 1.240168f;				

//线速度转角速度 rad/s
const float VEL2RAD = 1.0f/(WHELL_DIAMETER/2.0f);

//齿轮箱减速比;	
const float M3508_REDUCTION_RATIO = 13.733f;	
/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
Enum_Chassis_Control_Type Class_Tricycle_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取速度X限制
 *
 * @return float 速度X限制
 */
float Class_Tricycle_Chassis::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

/**
 * @brief 获取速度Y限制
 *
 * @return float 速度Y限制
 */
float Class_Tricycle_Chassis::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

/**
 * @brief 获取角速度限制
 *
 * @return float 角速度限制
 */
float Class_Tricycle_Chassis::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
float Class_Tricycle_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
float Class_Tricycle_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标角速度
 *
 * @return float 目标角速度
 */
float Class_Tricycle_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}


/**
 * @brief 获取小陀螺角速度
 *
 * @return float 小陀螺角速度
 */
float Class_Tricycle_Chassis::Get_Spin_Omega()
{
    return (Spin_Omega);
}

/**
 * @brief 获取当前电机功率
 *
 * @return float 当前电机功率
 */
float Class_Tricycle_Chassis::Get_Now_Power()
{
    return (Now_Power);
}

/**
 * @brief 获取当前舵向电机功率
 *
 * @return float 当前舵向电机功率
 */
float Class_Tricycle_Chassis::Get_Now_Steer_Power()
{
    return (Now_Steer_Power);
}

/**
 * @brief 获取可使用的舵向电机功率
 *
 * @return float 当前舵向电机功率
 */
float Class_Tricycle_Chassis::Get_Target_Steer_Power()
{
    return (Target_Steer_Power);
}

/**
 * @brief 获取当前轮向电机功率
 *
 * @return float 当前轮向电机功率
 */
float Class_Tricycle_Chassis::Get_Now_Wheel_Power()
{
    return (Now_Wheel_Power);
}

/**
 * @brief 获取可使用的轮向电机功率
 *
 * @return float 可使用的轮向电机功率
 */
float Class_Tricycle_Chassis::Get_Target_Wheel_Power()
{
    return (Target_Wheel_Power);
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
void Class_Tricycle_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
void Class_Tricycle_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
void Class_Tricycle_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
void Class_Tricycle_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定当前速度X
 *
 * @param __Now_Velocity_X 当前速度X
 */
void Class_Tricycle_Chassis::Set_Now_Velocity_X(float __Now_Velocity_X)
{
    Now_Velocity_X = __Now_Velocity_X;
}

/**
 * @brief 设定当前速度Y
 *
 * @param __Now_Velocity_Y 当前速度Y
 */
void Class_Tricycle_Chassis::Set_Now_Velocity_Y(float __Now_Velocity_Y)
{
    Now_Velocity_Y = __Now_Velocity_Y;
}

/**
 * @brief 设定当前角速度
 *
 * @param __Now_Omega 当前角速度
 */
void Class_Tricycle_Chassis::Set_Now_Omega(float __Velocity_Y_Max)
{
    Now_Omega = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大X速度
 *
 * @param __Velocity_Y_Max 输入
 */
void Class_Tricycle_Chassis::Set_Velocity_Y_Max(float __Velocity_Y_Max)
{
    Velocity_Y_Max = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大Y速度
 *
 * @param __Velocity_X_Max 输入
 */
void Class_Tricycle_Chassis::Set_Velocity_X_Max(float __Velocity_X_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
}


#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
