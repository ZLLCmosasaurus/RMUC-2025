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

enum Enum_Dart_FSM_Control_Status
{
    Dart_Init_Status = 0,
    Dart_Ready_Status,
    Dart_First_Status,
    Dart_Second_Status,
    Dart_Third_Status,
    Dart_Fourth_Status,
    Dart_Disaable_Status
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

class Class_FSM_Dart_Control : public Class_FSM
{
public:
    Class_Chariot *Chariot;

    // Enum_Dart_FSM_Control_Status Now_Status_Serial;

    // void Init(uint8_t __Status_Number, Enum_Dart_FSM_Control_Status __Now_Status_Serial)

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
        
        void TIM_Control_Callback();

        void TIM_Calculate_PeriodElapsedCallback();
        void TIM5msMod10_Alive_PeriodElapsedCallback();
        
        //迷你主机状态
        Enum_MiniPC_Status MiniPC_Status = MiniPC_Status_DISABLE;

protected:

        //遥控器拨动的死区, 0~1
        float DR16_Dead_Zone;
        //DR16云台yaw灵敏度系数(0.001PI表示yaw速度最大时为1rad/s)
        float DR16_Yaw_Angle_Resolution = 0.005f * PI * 57.29577951308232;
        //内部变量
        //拨盘发射标志位
        uint16_t Shoot_Cnt = 0;
        //读变量
        float True_Mouse_X;
        float True_Mouse_Y;
        float True_Mouse_Z;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
