/**
 * @file crt_gimbal.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 云台电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef CRT_GIMBAL_H
#define CRT_GIMBAL_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_djimotor.h"
#include "dvc_witahrs.h"
#include "dvc_lkmotor.h"
#include "dvc_minipc.h"
#include "dvc_imu.h"
#include "alg_fsm.h"
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
/**
 * @brief 吊射模式使能状态
 *
 */
enum Enum_Gimbal_Launch_Mode :uint8_t
{
    Launch_Disable = 0,
    Launch_Enable,
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
    Class_PID PID_Yaw_Encoder_Angle;
    Class_PID PID_Yaw_Encoder_Omega;

    inline float Get_Trer_Rad_Yaw();
    inline float Get_True_Gyro_Yaw();
    inline float Get_True_Angle_Yaw();
    inline float Get_True_Angle_Yaw_From_Encoder();

    void Transform_Angle();
    void Transform_EmcoderAngle_To_TrueAngle(); // 0-360°转为-180°至180°
    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //常量

    //内部变量
    //IMU获取的欧拉角
    float True_Rad_Yaw = 0.0f;
    float True_Angle_Yaw = 0.0f;
    float True_Gyro_Yaw = 0.0f;
    //编码器获取的相对角度值
    float EmcoderAngle_To_TrueAngle = 0.0f;
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

float Class_Gimbal_Yaw_Motor_GM6020::Get_True_Angle_Yaw_From_Encoder()
{
    return (EmcoderAngle_To_TrueAngle);
}

/**
 * @brief Specialized, pitch轴电机类
 *
 */
class Class_Gimbal_Pitch_Motor_M2006 : public Class_DJI_Motor_C610
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

float Class_Gimbal_Pitch_Motor_M2006::Get_True_Rad_Pitch()
{
    return (True_Rad_Pitch);
}

float Class_Gimbal_Pitch_Motor_M2006::Get_True_Angle_Pitch()
{
    return (True_Angle_Pitch);
}

float Class_Gimbal_Pitch_Motor_M2006::Get_True_Gyro_Pitch()
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
    Class_Gimbal_Yaw_Motor_GM6020 Motor_Yaw;
    // pitch轴电机
    Class_Gimbal_Pitch_Motor_M2006 Motor_Pitch;

    void Init();

    inline float Get_Target_Yaw_Angle();
    inline float Get_Target_Yaw_Encoder_Angle();
    inline float Get_Target_Pitch_Angle();
    inline Enum_Gimbal_Control_Type Get_Gimbal_Control_Type();
    inline Enum_Gimbal_Launch_Mode Get_Launch_Mode();
    inline void Set_Launch_Mode(Enum_Gimbal_Launch_Mode __Launch_Mode);
    inline void Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);
    inline void Set_Target_Yaw_Angle(float __Target_Yaw_Angle);
    inline void Set_Target_Yaw_Encoder_Angle(float __Target_Yaw_Encoder_Angle);
    inline void Set_Target_Pitch_Angle(float __Target_Pitch_Angle);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //常量
    // yaw轴最小值
    float Min_Yaw_Angle = - 180.0f;
    // yaw轴最大值
    float Max_Yaw_Angle = 180.0f;

    //yaw总角度
    float Yaw_Total_Angle;
    float Yaw_Half_Turns;

    // pitch轴最小值
    float Min_Pitch_Angle = -44.0f;
    // pitch轴最大值
    float Max_Pitch_Angle = 5.0f ;

    //内部变量 

    //读变量

    //写变量

    //云台状态
    Enum_Gimbal_Control_Type Gimbal_Control_Type = Gimbal_Control_Type_DISABLE ;
    Enum_Gimbal_Launch_Mode Launch_Mode = Launch_Disable;
    //读写变量

    // yaw轴角度
    float Target_Yaw_Angle = 0.0f;//IMU获取的欧拉角
    float Target_Yaw_Encoder_Angle = 0.0f;//编码器获取的相对角度值
    // pitch轴角度
    float Target_Pitch_Angle = 0.0f;
    //图传roll轴角度
    float Target_Image_Roll_Angle = 0.0f;
    //图传pitch轴角度
    float Target_Image_Pitch_Angle = 0.0f;
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
float Class_Gimbal::Get_Target_Yaw_Angle()
{
    return (Target_Yaw_Angle);
}
float Class_Gimbal::Get_Target_Yaw_Encoder_Angle()
{
    return (Target_Yaw_Encoder_Angle);
}
/**
 * @brief 获取pitch轴角度
 *
 * @return float pitch轴角度
 */
float Class_Gimbal::Get_Target_Pitch_Angle()
{
    return (Target_Pitch_Angle);
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

Enum_Gimbal_Launch_Mode Class_Gimbal::Get_Launch_Mode()
{
    return (Launch_Mode);
}
void Class_Gimbal::Set_Launch_Mode(Enum_Gimbal_Launch_Mode __Launch_Mode)
{
    Launch_Mode = __Launch_Mode;
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
void Class_Gimbal::Set_Target_Yaw_Angle(float __Target_Yaw_Angle)
{
    Target_Yaw_Angle = __Target_Yaw_Angle;
}
void Class_Gimbal::Set_Target_Yaw_Encoder_Angle(float __Target_Yaw_Encoder_Angle)
{
    Target_Yaw_Encoder_Angle = __Target_Yaw_Encoder_Angle;
}
/**
 * @brief 设定pitch轴角度
 *
 */
void Class_Gimbal::Set_Target_Pitch_Angle(float __Target_Pitch_Angle)
{
    Target_Pitch_Angle = __Target_Pitch_Angle;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
