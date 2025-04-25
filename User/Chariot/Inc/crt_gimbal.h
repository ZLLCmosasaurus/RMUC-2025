/**
 * @file crt_gimbal.h
 * @author lez by wanghongxi
 * @brief 云台
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef CRT_GIMBAL_H
#define CRT_GIMBAL_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_djimotor.h"
#include "dvc_minipc.h"
#include "dvc_imu.h"
#include "dvc_lkmotor.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/


/**
 * @brief 云台控制类型
 *
 */
enum Enum_Gimbal_Control_Type :uint8_t
{
    Gimbal_Control_Type_DISABLE = 0,
    Gimbal_Control_Type_NORMAL,
    Gimbal_Control_Type_MINIPC,
};

enum Enum_Motor_Yaw_Type :uint8_t
{
    Yaw_A = 0,
    Yaw_B,
};

struct IMU_Data
{
    float Pitch;
    float Roll;
    float Yaw;
    float Omega_X;
    float Omega_Y;
    float Omega_Z;
};

/**
 * @brief Specialized, yaw轴电机类
 *
 */
class Class_Gimbal_Yaw_Motor_GM6020 : public Class_DJI_Motor_GM6020
{
public:
    //陀螺仪获取云台角速度
    Class_IMU *IMU;

    inline float Get_Trer_Rad_Yaw();
    inline float Get_True_Gyro_Yaw();
    inline float Get_True_Angle_Yaw();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //常量

    //内部变量
    float True_Rad_Yaw = 0.0f;
    float True_Angle_Yaw = 0.0f;
    float True_Gyro_Yaw = 0.0f;
    //读变量

    //写变量

    //读写变量

    //内部函数
};

float Class_Gimbal_Yaw_Motor_GM6020::Get_Trer_Rad_Yaw()
{
    return (True_Rad_Yaw);
} 

float Class_Gimbal_Yaw_Motor_GM6020::Get_True_Gyro_Yaw()
{
    return (True_Gyro_Yaw);
}

float Class_Gimbal_Yaw_Motor_GM6020::Get_True_Angle_Yaw()
{
    return (True_Angle_Yaw);
}

/**
 * @brief Specialized, pitch轴电机类
 *
 */
class Class_Gimbal_Pitch_Motor_GM6020 : public Class_DJI_Motor_GM6020
{
public:
    //陀螺仪获取云台角速度
    Class_IMU* IMU;


    inline float Get_True_Rad_Pitch();
    inline float Get_True_Gyro_Pitch();
    inline float Get_True_Angle_Pitch();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关变量

    //常量
    

    // 重力补偿
    float Gravity_Compensate = 0.0f;

    //内部变量
    float True_Rad_Pitch = 0.0f;
    float True_Angle_Pitch = 0.0f;
    float True_Gyro_Pitch = 0.0f;
    //读变量

    //写变量

    //读写变量

    //内部函数
};

float Class_Gimbal_Pitch_Motor_GM6020::Get_True_Rad_Pitch()
{
    return (True_Rad_Pitch);
}

float Class_Gimbal_Pitch_Motor_GM6020::Get_True_Angle_Pitch()
{
    return (True_Angle_Pitch);
}

float Class_Gimbal_Pitch_Motor_GM6020::Get_True_Gyro_Pitch()
{
    return (True_Gyro_Pitch);
}


/**
 * @brief Specialized, pitch轴电机类
 *
 */
class Class_Gimbal_Pitch_Motor_LK6010 : public Class_LK_Motor
{
public:
    //陀螺仪获取云台角速度
    Class_IMU* IMU;
    
    inline float Get_True_Rad_Pitch();
    inline float Get_True_Gyro_Pitch();
    inline float Get_True_Angle_Pitch();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关变量

    //常量

    // 重力补偿
    float Gravity_Compensate = 0.0f;

    //内部变量 
    float True_Rad_Pitch = 0.0f;
    float True_Angle_Pitch = 0.0f;
    float True_Gyro_Pitch = 0.0f;
    //读变量

    //写变量

    //读写变量

    //内部函数
};

float Class_Gimbal_Pitch_Motor_LK6010::Get_True_Rad_Pitch()
{
    return (True_Rad_Pitch);
}

float Class_Gimbal_Pitch_Motor_LK6010::Get_True_Angle_Pitch()
{
    return (True_Angle_Pitch);
}

float Class_Gimbal_Pitch_Motor_LK6010::Get_True_Gyro_Pitch()
{
    return (True_Gyro_Pitch);

}

/**
 * @brief Specialized, 云台类
 *
 */
class Class_Gimbal
{
public:

    //imu对象
    Class_IMU Boardc_BMI;

    Class_MiniPC *MiniPC;

    /*后期yaw pitch这两个类要换成其父类，大疆电机类*/

    // yaw轴电机
    Class_DJI_Motor_GM6020 Motor_Yaw_A;
    Class_DJI_Motor_GM6020 Motor_Yaw_B;
    Class_LK_Motor Motor_Main_Yaw;

    // pitch轴电机
    Class_DJI_Motor_GM6020 Motor_Pitch_A;
    Class_DJI_Motor_GM6020 Motor_Pitch_B;

    void Init();

    inline float Get_Target_Yaw_Angle();
    inline float Get_Target_Yaw_Angle_A();
    inline float Get_Target_Yaw_Angle_B();  
    inline float Get_Target_Pitch_Angle_A();
    inline float Get_Target_Pitch_Angle_B();
    inline float Get_True_Angle_Yaw_A();
    inline float Get_True_Angle_Yaw_B();
    inline float Get_True_Angle_Pitch_A();
    inline float Get_True_Angle_Pitch_B();
    inline float Get_True_Angle_Yaw_Main();
    inline Enum_Gimbal_Control_Type Get_Gimbal_Control_Type();

    inline void Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);
    inline void Set_Target_Yaw_Angle(float __Target_Yaw_Angle);
    inline void Set_Target_Yaw_Angle_A(float __Target_Yaw_Angle_A);
    inline void Set_Target_Yaw_Angle_B(float __Target_Yaw_Angle_B);
    inline void Set_Target_Pitch_Angle_A(float __Target_Pitch_Angle_A);
    inline void Set_Target_Pitch_Angle_B(float __Target_Pitch_Angle_B);
    inline void Set_True_Angle_Yaw_A(float __True_Angle_Yaw_A);
    inline void Set_True_Angle_Yaw_B(float __True_Angle_Yaw_B);
    inline void Set_True_Angle_Pitch_A(float __True_Angle_Pitch_A);
    inline void Set_True_Angle_Pitch_B(float __True_Angle_Pitch_B);
    inline void Set_True_Angle_Yaw_Main(float __True_Angle_Yaw_Main);

    void TIM_Calculate_PeriodElapsedCallback();
    void Yaw_Angle_Transform_A();
    void Yaw_Angle_Transform_B();
    void Pitch_Angle_Transform_A();
    void Pitch_Angle_Transform_B();
    void Yaw_Angle_Transform_Main();
    void Control_Update_Main();
    void Yaw_Angle_Limit(Enum_Motor_Yaw_Type Motor_Yaw_Type);
    void MiniPC_Update();
    void PID_Update();
    void Limit_Update();
    float Adjust_Target(float Target,float Now);

    IMU_Data IMU_Data_A;
    IMU_Data IMU_Data_B;

protected:
    //初始化相关常量
    float Gimbal_Head_Angle;
    //常量
    float CRUISE_SPEED_YAW = 100.f;
    float CRUISE_SPEED_PITCH = 70.f;
    // yaw轴最小值
    float Min_Yaw_Angle = - 180.0f;
    float Min_Yaw_Angle_A = - 180.0f;
    float Min_Yaw_Angle_B = - 180.0f;
    // yaw轴最大值
    float Max_Yaw_Angle = 180.0f;
    float Max_Yaw_Angle_A = 180.0f;
    float Max_Yaw_Angle_B = 180.0f;

    //yaw总角度
    float Yaw_Total_Angle;
    float Yaw_Half_Turns;

    // pitch轴最小值
    float Min_Pitch_Angle = -15.0f;
    // pitch轴最大值
    float Max_Pitch_Angle = 25.0f ; //多10°

    //内部变量 
    float True_Angle_Yaw_A = 0.0f;
    float True_Angle_Yaw_B = 0.0f;
    float True_Angle_Pitch_A = 0.0f;
    float True_Angle_Pitch_B = 0.0f;
    float True_Angle_Yaw_Main = 0.0f;

    uint16_t A_Cruise_Flag = 0,B_Cruise_Flag = 0;
    uint16_t A_Invert_Flag = 0,B_Invert_Flag = 0;

    //读变量

    //写变量

    //云台状态
    Enum_Gimbal_Control_Type Gimbal_Control_Type = Gimbal_Control_Type_DISABLE ;

    Enum_Motor_Yaw_Type Motor_Yaw_Type = Yaw_A;

    //读写变量
    volatile float pre_angle_main = 0.f;
    // yaw轴角度
    float Target_Yaw_Angle = 0.0f;
    float Target_Yaw_Angle_A = 0.0f;
    float Target_Yaw_Angle_B = 0.0f;
    // pitch轴角度
    float Target_Pitch_Angle_A = 0.0f;
    float Target_Pitch_Angle_B = 0.0f;

    //内部函数

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/



/**
 * @brief 获取yaw轴角度
 *
 * @return float yaw轴角度
 */
float Class_Gimbal::Get_Target_Yaw_Angle_A()
{
    return (Target_Yaw_Angle_A);
}

/**
 * @brief 获取yaw轴角度
 *
 * @return float yaw轴角度
 */
float Class_Gimbal::Get_Target_Yaw_Angle()
{
    return (Target_Yaw_Angle);
}

float Class_Gimbal::Get_Target_Yaw_Angle_B()
{
    return (Target_Yaw_Angle_B);
}

/**
 * @brief 获取pitch轴角度
 *
 * @return float pitch轴角度
 */
float Class_Gimbal::Get_Target_Pitch_Angle_A()
{
    return (Target_Pitch_Angle_A);
}

float Class_Gimbal::Get_Target_Pitch_Angle_B()
{
    return (Target_Pitch_Angle_B);
}

float Class_Gimbal::Get_True_Angle_Yaw_A()
{
    return (True_Angle_Yaw_A);
}

float Class_Gimbal::Get_True_Angle_Yaw_B()
{
    return (True_Angle_Yaw_B);
}

float Class_Gimbal::Get_True_Angle_Pitch_A()
{
    return (True_Angle_Pitch_A);
}

float Class_Gimbal::Get_True_Angle_Pitch_B()
{
    return (True_Angle_Pitch_B);
}

float Class_Gimbal::Get_True_Angle_Yaw_Main()
{
    return (True_Angle_Yaw_Main);
}

/**
 * @brief 获取云台控制类型
 *
 * @return Enum_Gimbal_Control_Type 获取云台控制类型
 */
Enum_Gimbal_Control_Type Class_Gimbal::Get_Gimbal_Control_Type()
{
    return (Gimbal_Control_Type);
}

/**
 * @brief 设定云台状态
 *
 * @param __Gimbal_Control_Type 云台状态
 */
void Class_Gimbal::Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
{
    Gimbal_Control_Type = __Gimbal_Control_Type;
}

/**
 * @brief 设定yaw轴角度
 *
 */
void Class_Gimbal::Set_Target_Yaw_Angle_A(float __Target_Yaw_Angle_A)
{
    Target_Yaw_Angle_A = __Target_Yaw_Angle_A;
}

void Class_Gimbal::Set_Target_Yaw_Angle_B(float __Target_Yaw_Angle_B)
{
    Target_Yaw_Angle_B = __Target_Yaw_Angle_B;
}   

/**
 * @brief 设定yaw轴角度
 *
 */
void Class_Gimbal::Set_Target_Yaw_Angle(float __Target_Yaw_Angle)
{
    Target_Yaw_Angle = __Target_Yaw_Angle;
}

/**
 * @brief 设定pitch轴角度
 *
 */
void Class_Gimbal::Set_Target_Pitch_Angle_A(float __Target_Pitch_Angle_A)
{
    Target_Pitch_Angle_A = __Target_Pitch_Angle_A;
}

void Class_Gimbal::Set_Target_Pitch_Angle_B(float __Target_Pitch_Angle_B)
{
    Target_Pitch_Angle_B = __Target_Pitch_Angle_B;
}

void Class_Gimbal::Set_True_Angle_Yaw_Main(float __True_Angle_Yaw_Main)
{
    True_Angle_Yaw_Main = __True_Angle_Yaw_Main;
}

void Class_Gimbal::Set_True_Angle_Yaw_A(float __True_Angle_Yaw_A)
{
    True_Angle_Yaw_A = __True_Angle_Yaw_A;
}

void Class_Gimbal::Set_True_Angle_Yaw_B(float __True_Angle_Yaw_B)   
{
    True_Angle_Yaw_B = __True_Angle_Yaw_B;
}

void Class_Gimbal::Set_True_Angle_Pitch_A(float __True_Angle_Pitch_A)
{
    True_Angle_Pitch_A = __True_Angle_Pitch_A;
}

void Class_Gimbal::Set_True_Angle_Pitch_B(float __True_Angle_Pitch_B)
{
    True_Angle_Pitch_B = __True_Angle_Pitch_B;
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
