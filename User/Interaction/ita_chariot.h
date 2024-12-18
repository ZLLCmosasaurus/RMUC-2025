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

#include "alg_fsm.h"
#include "dvc_dr16.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"
#include "dvc_minipc.h"
#include "dvc_TensionMeter.h"
#include "dvc_servo.h"
#include "tsk_config_and_callback.h"

/* Exported macros -----------------------------------------------------------*/
class Class_Chariot;
/* Exported types ------------------------------------------------------------*/


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

        // yaw电机
        Class_DJI_Motor_GM6020 Motor_Yaw;

        // 下装填电机
        Class_DJI_Motor_C610 Motor_down;

        // 上装填电机
        Class_DJI_Motor_C610 Motor_up;

        // 左右装填电机
        Class_DJI_Motor_C620 Motor_left;
        Class_DJI_Motor_C620 Motor_right;

        // 拉力计
        Class_TensionMeter Tension_Meter;

        // 四个装填舵机
        Class_Servo Servo_Load_1;
        Class_Servo Servo_Load_2;
        Class_Servo Servo_Load_3;
        Class_Servo Servo_Load_4;

        // 一个扳机舵机
        Class_Servo Servo_Trigger;

        //裁判系统
        Class_Referee Referee;
        
        //遥控器
        Class_DR16 DR16;

        //上位机
        //Class_MiniPC MiniPC;

        //遥控器离线保护控制状态机
        Class_FSM_Alive_Control FSM_Alive_Control;
        friend class Class_FSM_Alive_Control;

        void Init(float __DR16_Dead_Zone = 0);

        inline void DR16_Offline_Cnt_Plus();
        inline uint16_t Get_DR16_Offline_Cnt();
        inline void Clear_DR16_Offline_Cnt();
        
        void TIM_Control_Callback();

        void TIM_Calculate_PeriodElapsedCallback();
        void TIM5msMod10_Alive_PeriodElapsedCallback();
        
        //迷你主机状态
        Enum_MiniPC_Status MiniPC_Status = MiniPC_Status_DISABLE;

protected:

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
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
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

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
