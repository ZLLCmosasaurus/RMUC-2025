/**
 * @file ita_chariot.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef TSK_INTERACTION_H
#define TSK_INTERACTION_H


/* Includes ------------------------------------------------------------------*/

#include "dvc_dr16.h"
#include "crt_gimbal.h"
#include "crt_booster.h"
#include "dvc_imu.h"
#include "tsk_config_and_callback.h"
#include "dvc_supercap.h"
#include "crt_chassis.h"
#include "config.h"
#include "crt_image.h"
#include "dvc_VT13.h"
/* Exported macros -----------------------------------------------------------*/
class Class_Chariot;
extern Class_Chariot chariot;
/* Exported types ------------------------------------------------------------*/

/**
 * @brief 云台Pitch状态枚举
 *
 */
enum Enum_Pitch_Control_Status 
{
    Pitch_Status_Control_Free = 0, 
    Pitch_Status_Control_Lock ,
};

enum Enum_MinPC_Aim_Status
{
    MinPC_Aim_Status_DISABLE = 0,
    MinPC_Aim_Status_ENABLE,
};

/**
 * @brief 摩擦轮状态
 *
 */
enum Enum_Fric_Status :uint8_t
{
    Fric_Status_CLOSE = 0,
    Fric_Status_OPEN,
};


/**
 * @brief 弹舱状态类型
 *
 */
enum Enum_Bulletcap_Status :uint8_t
{
    Bulletcap_Status_CLOSE = 0,
    Bulletcap_Status_OPEN,
};


/**
 * @brief 底盘通讯状态
 *
 */
enum Enum_Chassis_Status
{
    Chassis_Status_DISABLE = 0,
    Chassis_Status_ENABLE,
};

/**
 * @brief 云台通讯状态
 *
 */
enum Enum_Gimbal_Status
{
    Gimbal_Status_DISABLE = 0,
    Gimbal_Status_ENABLE,
};

/**
 * @brief DR16控制数据来源
 *
 */
enum Enum_DR16_Control_Type
{
    DR16_Control_Type_REMOTE = 0,
    DR16_Control_Type_KEYBOARD,
    DR16_Control_Type_NONE,
};

/**
 * @brief VT13控制数据来源
 *
 */
enum Enum_VT13_Control_Type
{
    VT13_Control_Type_REMOTE = 0,
    VT13_Control_Type_KEYBOARD,

    VT13_Control_Type_NONE,
};
/**
 * @brief 机器人是否离线 控制模式有限自动机
 *
 */
class Class_FSM_Alive_Control : public Class_FSM
{
public:
    Class_Chariot *Chariot;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

class Class_FSM_Alive_Control_VT13 : public Class_FSM
{
    public:
    uint8_t Start_Flag = 0;     //记录第一次上电开机，以初始化状态
    Class_Chariot *Chariot;

    void Reload_TIM_Status_PeriodElapsedCallback();
};
/**
 * @brief 控制对象
 *
 */
class Class_Chariot
{
public:
    #ifdef CHASSIS
        float Chassis_Coordinate_System_Angle_Rad;
        //获取yaw电机编码器值 用于底盘和云台坐标系的转换
        //底盘随动PID环
        Class_DJI_Motor_GM6020 Motor_Yaw;
        Class_PID PID_Chassis_Fllow;
        Class_PID PID_Chassis_Buffer_Power;
    #endif 

        //裁判系统
        Class_Referee Referee;
        //底盘
        Class_Streeing_Chassis Chassis;

    #ifdef GIMBAL
        //遥控器
        Class_DR16 DR16;

        Class_VT13 VT13;
        //上位机
        Class_MiniPC MiniPC;
        //云台
        Class_Gimbal Gimbal;
        //发射机构
        Class_Booster Booster;
        //图传
        Class_Image Image;
        //遥控器离线保护控制状态机
        Class_FSM_Alive_Control FSM_Alive_Control;
        friend class Class_FSM_Alive_Control;

        Class_FSM_Alive_Control_VT13 FSM_Alive_Control_VT13;
        friend class Class_FSM_Alive_Control_VT13;
    #endif

    void Init(float __DR16_Dead_Zone = 0);
    
    #ifdef CHASSIS
        float Get_Chassis_Coordinate_System_Angle_Rad();
        inline float Get_Gimbal_Yaw_IMU_Angle();
        inline void Set_Gimbal_Yaw_Angle(float __Angle);
        inline void  Set_Gimbal_Pitch_Angle(float __Angle);
        inline void Set_Chassis_Reference_Angle(float __Reference_Angle);
        inline void Process_Chassis_Logic_Direction();
        void CAN_Chassis_Rx_Gimbal_Callback();
        void CAN_Chassis_Tx_Gimbal_Callback();
        void TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
        void CAN_Chassis_Tx_Streeing_Wheel_Callback();
        void CAN_Chassis_Tx_Max_Power_Callback();
        void Chariot_Referee_UI_Tx_Callback(Enum_Referee_UI_Refresh_Status __Referee_UI_Refresh_Status);
        void Control_Chassis_Omega_TIM_PeriodElapsedCallback();
    #elif defined(GIMBAL)

        inline void DR16_Offline_Cnt_Plus();

        inline uint16_t Get_DR16_Offline_Cnt();
        inline void Clear_DR16_Offline_Cnt();

        inline Enum_Chassis_Control_Type Get_Pre_Chassis_Control_Type();
        inline Enum_Gimbal_Control_Type Get_Pre_Gimbal_Control_Type();
        inline Enum_Booster_Control_Type Get_Pre_Booster_Control_Type();

        inline void Set_Pre_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
        inline void Set_Pre_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);
        inline void Set_Pre_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type);

        inline Enum_Chassis_Status Get_Chassis_Status();
        inline Enum_DR16_Control_Type Get_DR16_Control_Type();
        inline Enum_VT13_Control_Type Get_VT13_Control_Type();

        void CAN_Gimbal_Rx_Chassis_Callback(uint8_t *Rx_Data);
        void CAN_Gimbal_Tx_Chassis_Callback();
        
        void TIM_Control_Callback();

        void TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback();
    #endif

    void TIM_Calculate_PeriodElapsedCallback();
    void TIM_Unline_Protect_PeriodElapsedCallback();
    void TIM1msMod50_Alive_PeriodElapsedCallback();
    
    //底盘云台通讯变量
    //一键掉头
    Enum_Chassis_Logics_Direction Chassis_Logics_Direction = Chassis_Logic_Direction_Positive;
    //冲刺
    Enum_Sprint_Status Sprint_Status = Sprint_Status_DISABLE;
    //yaw编码开关
    Enum_Yaw_Encoder_Control_Status Yaw_Encoder_Control_Status = Yaw_Encoder_Control_Status_Disable;
    //摩擦轮开关
    Enum_Fric_Status Fric_Status = Fric_Status_CLOSE;
    //超级电容超级放电状态
    Enum_Supercap_Control_Status  Supercap_Control_Status = Supercap_Control_Status_DISABLE;
    //迷你主机状态
    Enum_MiniPC_Status MiniPC_Status = MiniPC_Status_DISABLE;
    //裁判系统UI刷新状态
    Enum_Referee_UI_Refresh_Status Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_DISABLE;
    //底盘云台通讯数据
    float Gimbal_Tx_Pitch_Angle = 0;
    float Shoot_Speed = 0;
protected:

    //pitch控制状态 锁定和自由控制
    Enum_Pitch_Control_Status  Pitch_Control_Status = Pitch_Status_Control_Free;
    
    //初始化相关常量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object = &CAN1_Manage_Object;

    #ifdef CHASSIS
        //底盘标定参考正方向角度(数据来源yaw电机)
        float Reference_Angle = 1.07800508f;
        //小陀螺云台坐标系稳定偏转角度 用于矫正
        float Offset_Angle = 0.0f;//12.0f;
        //底盘转换后的角度（数据来源yaw电机）
        float Chassis_Angle;
        //获取云台的IMU yaw轴角度
        float Yaw_IMU_Angle;
        float Pitch_IMU_Angle;
        //写变量
        uint32_t Gimbal_Alive_Flag = 0;
        uint32_t Pre_Gimbal_Alive_Flag = 0;

        Enum_Gimbal_Status Gimbal_Status =  Gimbal_Status_DISABLE;
    #endif

    #ifdef GIMBAL
        //遥控器拨动的死区, 0~1
        float DR16_Dead_Zone;
        float VT13_Dead_Zone;
        //常量
        //键鼠模式按住shift 最大速度缩放系数
        float DR16_Mouse_Chassis_Shift = 2.0f;
        float VT13_Mouse_Chassis_Shift = 2.0f;
        //舵机占空比 默认关闭弹舱
        uint16_t Compare =400;
        //DR16底盘加速灵敏度系数(0.001表示底盘加速度最大为1m/s2)
        float DR16_Keyboard_Chassis_Speed_Resolution_Small = 0.001f;
        float VT13_Keyboard_Chassis_Speed_Resolution_Small = 0.001f;
        //DR16底盘减速灵敏度系数(0.001表示底盘加速度最大为1m/s2)
        float DR16_Keyboard_Chassis_Speed_Resolution_Big = 0.01f;
        float VT13_Keyboard_Chassis_Speed_Resolution_Big = 0.01f;
        //DR16云台yaw灵敏度系数(0.001PI表示yaw速度最大时为1rad/s)
        float DR16_Yaw_Angle_Resolution = 0.005f * PI * 57.29577951308232;
        float VT13_Yaw_Angle_Resolution = 0.005f * PI * 57.29577951308232;
        //DR16云台pitch灵敏度系数(0.001PI表示pitch速度最大时为1rad/s)
        float DR16_Pitch_Angle_Resolution = 0.003f * PI * 57.29577951308232;
        float VT13_Pitch_Angle_Resolution = 0.003f * PI * 57.29577951308232;

        //DR16云台yaw灵敏度系数(0.001PI表示yaw速度最大时为1rad/s)
        float DR16_Yaw_Resolution = 0.003f * PI;
        float VT13_Yaw_Resolution = 0.003f * PI;
        //DR16云台pitch灵敏度系数(0.001PI表示pitch速度最大时为1rad/s)
        float DR16_Pitch_Resolution = 0.003f * PI;
        float VT13_Pitch_Resolution = 0.003f * PI;
        //DR16鼠标云台yaw灵敏度系数, 不同鼠标不同参数
        float DR16_Mouse_Yaw_Angle_Resolution = 57.8*4.0f;
        float VT13_Mouse_Yaw_Angle_Resolution = 57.8*4.0f;
        //DR16鼠标云台pitch灵敏度系数, 不同鼠标不同参数
        float DR16_Mouse_Pitch_Angle_Resolution = 57.8f*2.0f;
        float VT13_Mouse_Pitch_Angle_Resolution = 57.8f*2.0f;
        //迷你主机云台pitch自瞄控制系数
        float MiniPC_Autoaiming_Yaw_Angle_Resolution = 0.003f;
        //迷你主机云台pitch自瞄控制系数
        float MiniPC_Autoaiming_Pitch_Angle_Resolution = 0.003f;

        //内部变量
        //遥控器离线计数
        uint16_t DR16_Offline_Cnt = 0;
        //拨盘发射标志位
        uint16_t Shoot_Cnt = 0;
        //读变量
        float True_Mouse_X;
        float True_Mouse_Y;
        float True_Mouse_Z;
        //写变量
        uint32_t Chassis_Alive_Flag = 0;
        uint32_t Pre_Chassis_Alive_Flag = 0;
        //读写变量
        Enum_Chassis_Status Chassis_Status = Chassis_Status_DISABLE;

        //底盘 云台 发射机构 前一帧控制类型
        Enum_Chassis_Control_Type Pre_Chassis_Control_Type = Chassis_Control_Type_FLLOW;
        Enum_Gimbal_Control_Type Pre_Gimbal_Control_Type = Gimbal_Control_Type_NORMAL;
        Enum_Booster_Control_Type Pre_Booster_Control_Type = Booster_Control_Type_CEASEFIRE;

        //单发连发标志位
        uint8_t Shoot_Flag = 0;
        //DR16控制数据来源
        Enum_DR16_Control_Type DR16_Control_Type = DR16_Control_Type_NONE;
         Enum_VT13_Control_Type VT13_Control_Type = VT13_Control_Type_NONE;
        //内部函数

        void Judge_DR16_Control_Type();

        void Control_Chassis();
        void Control_Gimbal();
        void Control_Booster();
        void Control_Image();

        void Transform_Mouse_Axis();
    #endif
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


#ifdef GIMBAL
    /**
     * @brief 获取底盘通讯状态
     * 
     * @return Enum_Chassis_Status 底盘通讯状态
     */
    Enum_Chassis_Status Class_Chariot::Get_Chassis_Status()
    {
        return (Chassis_Status);
    }

    /**
     * @brief 获取DR16控制数据来源
     * 
     * @return Enum_DR16_Control_Type DR16控制数据来源
     */

    Enum_DR16_Control_Type Class_Chariot::Get_DR16_Control_Type()
    {
        return (DR16_Control_Type);
    }

    /**
     * @brief 获取VT13控制数据来源
     * 
     * @return Enum_VT13_Control_Type VT13控制数据来源
     */
    Enum_VT13_Control_Type Class_Chariot::Get_VT13_Control_Type()
    {
        return (VT13_Control_Type);
    }
    /**
     * @brief 获取前一帧底盘控制类型
     * 
     * @return Enum_Chassis_Control_Type 前一帧底盘控制类型
     */

    Enum_Chassis_Control_Type Class_Chariot::Get_Pre_Chassis_Control_Type()
    {
        return (Pre_Chassis_Control_Type);
    }

    /**
     * @brief 获取前一帧云台控制类型
     * 
     * @return Enum_Gimbal_Control_Type 前一帧云台控制类型
     */

    Enum_Gimbal_Control_Type Class_Chariot::Get_Pre_Gimbal_Control_Type()
    {
        return (Pre_Gimbal_Control_Type);
    }

    /**
     * @brief 获取前一帧发射机构控制类型
     * 
     * @return Enum_Booster_Control_Type 前一帧发射机构控制类型
     */
    Enum_Booster_Control_Type Class_Chariot::Get_Pre_Booster_Control_Type()
    {
        return (Pre_Booster_Control_Type);
    }

    /**
     * @brief 设置前一帧底盘控制类型
     * 
     * @param __Chassis_Control_Type 前一帧底盘控制类型
     */
    void Class_Chariot::Set_Pre_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
    {
        Pre_Chassis_Control_Type = __Chassis_Control_Type;
    }

    /**
     * @brief 设置前一帧云台控制类型
     * 
     * @param __Gimbal_Control_Type 前一帧云台控制类型
     */
    void Class_Chariot::Set_Pre_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
    {
        Pre_Gimbal_Control_Type = __Gimbal_Control_Type;
    }

    /**
     * @brief 设置前一帧发射机构控制类型
     * 
     * @param __Booster_Control_Type 前一帧发射机构控制类型
     */
    void Class_Chariot::Set_Pre_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type)
    {
        Pre_Booster_Control_Type = __Booster_Control_Type;
    }

    /**
     * @brief DR16离线计数加一
     */
    void Class_Chariot::DR16_Offline_Cnt_Plus()
    {
        DR16_Offline_Cnt++;
    }

    /**
     * @brief 获取DR16离线计数
     * 
     * @return uint16_t DR16离线计数
     */
    uint16_t Class_Chariot::Get_DR16_Offline_Cnt()
    {
        return (DR16_Offline_Cnt);
    }

    /**
     * @brief DR16离线计数置0
     * 
     */
    void Class_Chariot::Clear_DR16_Offline_Cnt()
    {
        DR16_Offline_Cnt = 0;
    
    }


#endif
#ifdef CHASSIS
float Class_Chariot::Get_Gimbal_Yaw_IMU_Angle()
{
    return (Yaw_IMU_Angle);
}
void Class_Chariot::Set_Gimbal_Yaw_Angle(float __Angle)
{
    Yaw_IMU_Angle = __Angle;
}
void Class_Chariot::Set_Gimbal_Pitch_Angle(float __Angle)
{
    Pitch_IMU_Angle = __Angle;
}
/**
 * @brief 设置云台跟随偏移角度
 * 
 */
void Class_Chariot::Set_Chassis_Reference_Angle(float __Reference_Angle)
{
    Reference_Angle = __Reference_Angle;
}
/**
 * @brief 处理底盘逻辑方向，并在底盘坐标系下设置新的云台跟随偏移角度
 * 
 */
void Class_Chariot::Process_Chassis_Logic_Direction()
{
    static uint8_t Process_Complete_flag = 0;
    static float  Chassis_Logic_Direction_Positive_Angle = 0.0f;
    static float  Chassis_Logic_Direction_Negative_Angle = 0.0f;
    if(Process_Complete_flag == 0)
    {
        Chassis_Logic_Direction_Positive_Angle = Reference_Angle;
        Chassis_Logic_Direction_Negative_Angle = Reference_Angle - PI;
        Process_Complete_flag = 1;
    }

    // 如果底盘逻辑方向是正向
    if (Chassis_Logics_Direction == Chassis_Logic_Direction_Positive)
        Set_Chassis_Reference_Angle(Chassis_Logic_Direction_Positive_Angle); // 设置底盘参考角度为 Chassis_Logic_Direction_Positive_Angle
    else
        Set_Chassis_Reference_Angle(Chassis_Logic_Direction_Negative_Angle); // 设置底盘参考角度为 Chassis_Logic_Direction_Negative_Angle
}

#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
