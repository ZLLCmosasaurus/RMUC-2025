/**
 * @file crt_chassis.h
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

#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H
#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/

#include "alg_slope.h"
#include "dvc_sampler.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"
#include "alg_power_limit.h"
#include "dvc_supercap.h"
#include "alg_fsm.h"
#include "config.h"
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

/**
 * @brief Specialized, 三轮舵轮底盘类
 *
 */
//omnidirectional 全向轮
class Class_Tricycle_Chassis
{
public:

    //斜坡函数加减速速度X
    Class_Slope Slope_Velocity_X;
    //斜坡函数加减速速度Y
    Class_Slope Slope_Velocity_Y;
    //斜坡函数加减速角速度
    Class_Slope Slope_Omega;

    Class_Supercap Supercap;
    #ifdef POWER_LIMIT
    
    //功率限制
    Class_Power_Limit Power_Limit;
    
    
    #endif
    //裁判系统
    Class_Referee *Referee;

    //下方转动电机
    Class_DJI_Motor_C620 Motor_Wheel[4];

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

/*-------------------------------------------------舵轮底盘-------------------------------------------------------*/
struct Struct_Streeing_wheel
{
    float ChassisCoordinate_Angle;//底盘坐标系下的舵角度
    float  streeing_wheel_angle;//角度：rpm
    float streeing_wheel_speed;//线速度：米每秒 
    float streeing_wheel_omega;//角速度：rpm
};

//驻车状态机
//class  Class_FSM_Chassis_Stop : public Class_FSM
//{
//public:
//		Class_Streeing_Chassis *chassis;
//		void Reload_TIM_Status_PeriodElapsedCallback();
//};
/*
    舵轮底盘类
*/
//steering wheel舵轮底盘
class Class_Streeing_Chassis
{
public:
    Struct_Streeing_wheel wheel[4];
//		 Class_FSM_Chassis_Stop FSM_Chassis_Stop;
//    friend class Class_FSM_Chassis_Stop;
    //斜坡函数加减速速度X
    Class_Slope Slope_Velocity_X;
    //斜坡函数加减速速度Y
    Class_Slope Slope_Velocity_Y;
    //斜坡函数加减速角速度
    Class_Slope Slope_Omega;

    Class_Supercap Supercap;
    #ifdef POWER_LIMIT
    
    //功率限制
    Class_Power_Limit Power_Limit;
    
    
    #endif
    //裁判系统
    Class_Referee *Referee;

    void Init(float __Velocity_X_Max = 4.0f, float __Velocity_Y_Max = 4.0f, float __Omega_Max = 8.0f, float __Steer_Power_Ratio = 0.5f);

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
    inline void Set_Turn_Flag(uint8_t _flag);
    inline uint8_t Get_Turn_Flag();
    inline void Set_Velocity_Y_Max(float __Velocity_Y_Max);
    inline void Set_Velocity_X_Max(float __Velocity_X_Max);

    void TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status);

protected:
    //初始化相关常量
    uint8_t break_mode;
    //速度X限制
    float Velocity_X_Max;
    //速度Y限制
    float Velocity_Y_Max;
    //角速度限制
    float Omega_Max;
    //舵向电机功率上限比率
    float Steer_Power_Ratio = 0.5f;
    //底盘小陀螺模式角速度
    float Spin_Omega = 5.0f;
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
    float Target_Velocity_X = 0.0f;//规定X轴方向为：底盘左右平移方向 以右边为正方向
    //目标速度Y
    float Target_Velocity_Y = 0.0f;//规定Y轴方向为：底盘前进后退方向 以前边为正方向
    //目标角速度
    float Target_Omega = 0.0f;//规定：顺时针为正方向
    //当前速度X
    float Now_Velocity_X = 0.0f;
    //当前速度Y
    float Now_Velocity_Y = 0.0f;
    //当前角速度
    float Now_Omega = 0.0f;

    //内部函数
    void Speed_Resolution();
    void AGV_DirectiveMotor_TargetStatus_To_MotorAngle_In_ChassisCoordinate();
    void Speed_Limitation();
};
#define MAX_MOTOR_SPEED 500    
#define wheel_diameter 0.12f   // m
#define half_width   0.406f    //m     		//x方向为宽
#define half_length 0.406f     //m
#define ROTATION_CENTER_OFFSET 0.0f // 旋转中心位置偏移量，现在只有y方向上的偏移，且向y负方向偏移，这个偏移量为绝对值

#define THETA_A atan((half_length + ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_B atan((half_length + ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_C atan((half_length - ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_D atan((half_length - ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）

#define R_A half_width / cos(THETA_A) // 旋转中心与A舵轮的距离
#define R_B half_width / cos(THETA_B) // 旋转中心与B舵轮的距离
#define R_C half_width / cos(THETA_C) // 旋转中心与C舵轮的距离
#define R_D half_width / cos(THETA_D) // 旋转中心与D舵轮的距离

#define RAD_TO_8191 8191.0f / PI / 2
#define VEL2RPM 60.0f/PI/wheel_diameter // //rpm = vel*60/π/D    vel(m/s)->rpm
/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
Enum_Chassis_Control_Type Class_Streeing_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取速度X限制
 *
 * @return float 速度X限制
 */
float Class_Streeing_Chassis::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

/**
 * @brief 获取速度Y限制
 *
 * @return float 速度Y限制
 */
float Class_Streeing_Chassis::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

/**
 * @brief 获取角速度限制
 *
 * @return float 角速度限制
 */
float Class_Streeing_Chassis::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
float Class_Streeing_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
float Class_Streeing_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标角速度
 *
 * @return float 目标角速度
 */
float Class_Streeing_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}


/**
 * @brief 获取小陀螺角速度
 *
 * @return float 小陀螺角速度
 */
float Class_Streeing_Chassis::Get_Spin_Omega()
{
    return (Spin_Omega);
}

/**
 * @brief 获取当前电机功率
 *
 * @return float 当前电机功率
 */
float Class_Streeing_Chassis::Get_Now_Power()
{
    return (Now_Power);
}

/**
 * @brief 获取当前舵向电机功率
 *
 * @return float 当前舵向电机功率
 */
float Class_Streeing_Chassis::Get_Now_Steer_Power()
{
    return (Now_Steer_Power);
}

/**
 * @brief 获取可使用的舵向电机功率
 *
 * @return float 当前舵向电机功率
 */
float Class_Streeing_Chassis::Get_Target_Steer_Power()
{
    return (Target_Steer_Power);
}

/**
 * @brief 获取当前轮向电机功率
 *
 * @return float 当前轮向电机功率
 */
float Class_Streeing_Chassis::Get_Now_Wheel_Power()
{
    return (Now_Wheel_Power);
}

/**
 * @brief 获取可使用的轮向电机功率
 *
 * @return float 可使用的轮向电机功率
 */
float Class_Streeing_Chassis::Get_Target_Wheel_Power()
{
    return (Target_Wheel_Power);
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
void Class_Streeing_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
void Class_Streeing_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
void Class_Streeing_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
void Class_Streeing_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定当前速度X
 *
 * @param __Now_Velocity_X 当前速度X
 */
void Class_Streeing_Chassis::Set_Now_Velocity_X(float __Now_Velocity_X)
{
    Now_Velocity_X = __Now_Velocity_X;
}

/**
 * @brief 设定当前速度Y
 *
 * @param __Now_Velocity_Y 当前速度Y
 */
void Class_Streeing_Chassis::Set_Now_Velocity_Y(float __Now_Velocity_Y)
{
    Now_Velocity_Y = __Now_Velocity_Y;
}

/**
 * @brief 设定当前角速度
 *
 * @param __Now_Omega 当前角速度
 */
void Class_Streeing_Chassis::Set_Now_Omega(float __Velocity_Y_Max)
{
    Now_Omega = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大X速度
 *
 * @param __Velocity_Y_Max 输入
 */
void Class_Streeing_Chassis::Set_Velocity_Y_Max(float __Velocity_Y_Max)
{
    Velocity_Y_Max = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大Y速度
 *
 * @param __Velocity_X_Max 输入
 */
void Class_Streeing_Chassis::Set_Velocity_X_Max(float __Velocity_X_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
}


#endif
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
