/**
 * @file ita_chariot.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
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
#include "config.h"
#include "crt_information_platform.h"
/* Exported macros -----------------------------------------------------------*/
class Class_Chariot;
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
 * @brief DR16控制数据来源
 *
 */
enum Enum_DR16_Control_Type
{
    DR16_Control_Type_REMOTE = 0,
    DR16_Control_Type_KEYBOARD,
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

/**
 * @brief 控制对象
 *
 */
class Class_Chariot
{
public:
  
    #ifdef CHASSIS
        //获取yaw电机编码器值 用于底盘和云台坐标系的转换
        
        
        //底盘
        Class_Tricycle_Chassis Chassis;

        inline Enum_Chassis_Control_Type Get_Pre_Chassis_Control_Type();
        inline void Set_Pre_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);


    #endif
        
    #ifdef REFEREE
        //裁判系统
        Class_Referee Referee;
    #endif
        
    #ifdef CONTROLLER

    #ifdef DR16_REMOTE
        //遥控器
        Class_DR16 DR16;

        //遥控器离线保护控制状态机
        Class_FSM_Alive_Control FSM_Alive_Control;
        friend class Class_FSM_Alive_Control;

        inline void DR16_Offline_Cnt_Plus();
        inline uint16_t Get_DR16_Offline_Cnt();
        inline void Clear_DR16_Offline_Cnt();
        inline Enum_DR16_Control_Type Get_DR16_Control_Type();
        void TIM_Unline_Protect_PeriodElapsedCallback();

    #endif
    
    #endif
       
    #ifdef MINI_PC
        //上位机
        Class_MiniPC MiniPC;
    #endif
 
    #ifdef GIMBAL
        //云台
        Class_Gimbal Gimbal;

        Class_DJI_Motor_GM6020 Motor_Yaw;
        //底盘随动PID环
        Class_PID PID_Chassis_Fllow;

        void TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();

        inline Enum_Gimbal_Control_Type Get_Pre_Gimbal_Control_Type();
        inline void Set_Pre_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);
        
    #endif  
        
    #ifdef BOOSTER
        //发射机构
        Class_Booster Booster;

        
        inline Enum_Booster_Control_Type Get_Pre_Booster_Control_Type();
        inline void Set_Pre_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type);
    #endif

    #ifdef INFORMATION_PLATFORM

        Class_Information_Platform Information_Platform;

        #ifdef CHASSIS
            void CAN_Chassis_Rx_Gimbal_Callback();
            void TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
        #endif 
       
       #ifdef GIMBAL
            void CAN_Gimbal_Rx_Chassis_Callback();
            void TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback();
       #endif
            void Information_Platform_Update();

    //底盘云台通讯变量
    //冲刺
    Enum_Sprint_Status Sprint_Status = Sprint_Status_DISABLE;
    //弹仓开关
    Enum_Bulletcap_Status Bulletcap_Status = Bulletcap_Status_CLOSE;
    //摩擦轮开关
    Enum_Fric_Status Fric_Status = Fric_Status_CLOSE;
    //自瞄锁住状态
    Enum_MinPC_Aim_Status MiniPC_Aim_Status = MinPC_Aim_Status_DISABLE;
    //迷你主机状态
    Enum_MiniPC_Status MiniPC_Status = MiniPC_Status_DISABLE;
    //裁判系统UI刷新状态
    Enum_Referee_UI_Refresh_Status Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_DISABLE;
    //底盘云台通讯数据
    float Gimbal_Tx_Pitch_Angle = 0;
        
    #endif 
       
    void Init(float __DR16_Dead_Zone = 0);
    void TIM_Control_Callback();
    void TIM_Calculate_PeriodElapsedCallback();
    void TIM1msMod50_Alive_PeriodElapsedCallback();
    
    

protected:

    
    #ifdef CHASSIS
        

        Enum_Chassis_Control_Type Pre_Chassis_Control_Type = Chassis_Control_Type_DISABLE;
        
        void Control_Chassis();
    #endif
        
    #ifdef REFEREE

    #endif
        
    #ifdef CONTROLLER

    #ifdef DR16_REMOTE
        //遥控器拨动的死区, 0~1
        float DR16_Dead_Zone;
        //常量
        //键鼠模式按住shift 最大速度缩放系数
        float DR16_Mouse_Chassis_Shift = 2.0f;
        //舵机占空比 默认关闭弹舱
        uint16_t Compare =400;
        //DR16底盘加速灵敏度系数(0.001表示底盘加速度最大为1m/s2)
        float DR16_Keyboard_Chassis_Speed_Resolution_Small = 0.001f;
        //DR16底盘减速灵敏度系数(0.001表示底盘加速度最大为1m/s2)
        float DR16_Keyboard_Chassis_Speed_Resolution_Big = 0.01f;

        //DR16云台yaw灵敏度系数(0.001PI表示yaw速度最大时为1rad/s)
        float DR16_Yaw_Angle_Resolution = 0.005f * PI * 57.29577951308232;
        //DR16云台pitch灵敏度系数(0.001PI表示pitch速度最大时为1rad/s)
        float DR16_Pitch_Angle_Resolution = 0.003f * PI * 57.29577951308232;

        //DR16云台yaw灵敏度系数(0.001PI表示yaw速度最大时为1rad/s)
        float DR16_Yaw_Resolution = 0.003f * PI;
        //DR16云台pitch灵敏度系数(0.001PI表示pitch速度最大时为1rad/s)
        float DR16_Pitch_Resolution = 0.003f * PI;

        //DR16鼠标云台yaw灵敏度系数, 不同鼠标不同参数
        float DR16_Mouse_Yaw_Angle_Resolution = 57.8*4.0f;
        //DR16鼠标云台pitch灵敏度系数, 不同鼠标不同参数
        float DR16_Mouse_Pitch_Angle_Resolution = 57.8f;
        //遥控器离线计数
        uint16_t DR16_Offline_Cnt = 0;

        float True_Mouse_X;
        float True_Mouse_Y;
        float True_Mouse_Z;

        //DR16控制数据来源
        Enum_DR16_Control_Type DR16_Control_Type = DR16_Control_Type_REMOTE;
        void Judge_DR16_Control_Type();
        void Transform_Mouse_Axis();
    #endif
    
    #endif
       
    #ifdef MINI_PC
        //迷你主机云台pitch自瞄控制系数
        float MiniPC_Autoaiming_Yaw_Angle_Resolution = 0.003f;
        //迷你主机云台pitch自瞄控制系数
        float MiniPC_Autoaiming_Pitch_Angle_Resolution = 0.003f;
    #endif
 
    #ifdef GIMBAL
        //pitch控制状态 锁定和自由控制
        //底盘标定参考正方向角度(数据来源yaw电机)
        float Reference_Angle = 0.520019531f;
        //小陀螺云台坐标系稳定偏转角度 用于矫正
        float Offset_Angle = 0.0f;  //7.5°
        //底盘转换后的角度（数据来源yaw电机）
        float Chassis_Angle;
        Enum_Pitch_Control_Status  Pitch_Control_Status = Pitch_Status_Control_Free;
        Enum_Gimbal_Control_Type Pre_Gimbal_Control_Type = Gimbal_Control_Type_NORMAL;
        void Control_Gimbal();
        
    #endif  
        
    #ifdef BOOSTER
        uint16_t Shoot_Cnt = 0;
        Enum_Booster_Control_Type Pre_Booster_Control_Type = Booster_Control_Type_CEASEFIRE;

        //单发连发标志位
        uint8_t Shoot_Flag = 0;
        void Control_Booster();
    #endif

    #ifdef INFORMATION_PLATFORM

        #ifdef CHASSIS
            
        #endif 
       
        #ifdef GIMBAL
            
            
            void Control_Platform();
        #endif
        //绑定的CAN
        Struct_CAN_Manage_Object *CAN_Manage_Object = &CAN2_Manage_Object;
        
    #endif 
    
  
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


    #ifdef CHASSIS
    /**
     * @brief 设置前一帧底盘控制类型
     * 
     * @param __Chassis_Control_Type 前一帧底盘控制类型
     */
    void Class_Chariot::Set_Pre_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
    {
        Pre_Chassis_Control_Type = __Chassis_Control_Type;
    }
    #endif
        
    #ifdef REFEREE

    #endif
        
    #ifdef CONTROLLER

    #ifdef DR16_REMOTE
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
     * @brief 获取前一帧底盘控制类型
     * 
     * @return Enum_Chassis_Control_Type 前一帧底盘控制类型
     */
#ifdef CHASSIS
    Enum_Chassis_Control_Type Class_Chariot::Get_Pre_Chassis_Control_Type()
    {
        return (Pre_Chassis_Control_Type);
    }
#endif
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
    
    #endif
       
    #ifdef MINI_PC
      
    #endif
 
    #ifdef GIMBAL
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
     * @brief 设置前一帧云台控制类型
     * 
     * @param __Gimbal_Control_Type 前一帧云台控制类型
     */
    void Class_Chariot::Set_Pre_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
    {
        Pre_Gimbal_Control_Type = __Gimbal_Control_Type;
    }
   
        
    #endif  
        
    #ifdef BOOSTER
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
     * @brief 设置前一帧发射机构控制类型
     * 
     * @param __Booster_Control_Type 前一帧发射机构控制类型
     */
    void Class_Chariot::Set_Pre_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type)
    {
        Pre_Booster_Control_Type = __Booster_Control_Type;
    }  
    #endif

    #ifdef INFORMATION_PLATFORM

        #ifdef CHASSIS

        #endif 
       
        #ifdef GIMBAL
          
        #endif
       
    #endif 


    // #ifdef CHASSIS

    // #endif
        
    // #ifdef REFEREE

    // #endif
        
    // #ifdef CONTROLLER

    // #ifdef DR16_REMOTE


    // #endif
    
    // #endif
       
    // #ifdef MINI_PC
      
    // #endif
 
    // #ifdef GIMBAL
    
    // #endif  
        
    // #ifdef BOOSTER
 
    // #endif

    // #ifdef INFORMATION_PLATFORM

    //     #ifdef CHASSIS

    //     #endif 
       
    //     #ifdef GIMBAL
          
    //     #endif
       
    // #endif 
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
