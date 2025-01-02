/**
 * @file ita_chariot.h
 * @author lez by yssickjgd
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
#include "dvc_DebugControl.h"
#include "tsk_config_and_callback.h"

/* Exported macros -----------------------------------------------------------*/
class Class_Chariot;
/* Exported types ------------------------------------------------------------*/

enum Enum_Dart_FSM_Control_Status : uint8_t
{
    Dart_Init_Status = 0,
    Dart_Ready_Status,
    Dart_First_Status,
    Dart_Second_Status,
    Dart_Third_Status,
    Dart_Fourth_Status,
    Dart_Debug_Status,
    Dart_Disable_Status
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

    void Reload_TIM_Status_PeriodElapsedCallback();
};


/**
 * @brief 控制对象
 *
 */
class Class_Chariot
{
public:
        // 拉力环PID
        Class_PID PID_Tension;

        // yaw电机
        Class_DJI_Motor_GM6020 Motor_Yaw;

        // 上下装填电机
        Class_DJI_Motor_C610 Motor_Up;
        Class_DJI_Motor_C610 Motor_Down;

        // 左右装填电机
        Class_DJI_Motor_C620 Motor_Left;
        Class_DJI_Motor_C620 Motor_Right;

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

        //调试控制
        Class_DebugControl DebugControl;

        //上位机
        //Class_MiniPC MiniPC;

        //遥控器离线保护控制状态机
        Class_FSM_Alive_Control FSM_Alive_Control;
        friend class Class_FSM_Alive_Control;

        // Dart 控制状态机
        Class_FSM_Dart_Control FSM_Dart_Control;
        friend class Class_FSM_Dart_Control;

        void Init(float __DR16_Dead_Zone = 0);
        
        bool Calibrate();
        void Updata_Distance_Angle();
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

        // 推镖电机 丝杆比例系数
        float Radian_To_Diatance = 0.0f;
        // yaw电机减速比系数
        float Reduction_Ratio = 0.0f; 
        // 扳机舵机角度
        float Shoot_Angle_Trigger; //按下角度
        float Close_Angle_Trigger; //松开角度

        // 推镖电机distance
        float Now_Distance_Motor_Up;   
        float Tartget_Distance_Motor_Up;
        float Target_Speed_Motor_Up;
        // Yaw电机角度
        float Now_Angle_Yaw;
        float Target_Angle_Yaw;

        // 拉皮筋速度
        float Target_Speed_Motor_Left;
        float Target_Speed_Motor_Right;

        // 目标拉力
        float Target_Tension;

        //读变量
        
        // 校准完成标志位
        bool Calibration_Finish = false;

        // 电机校准目标角速度
        float Calibration_Motor_Yaw_Target_Omega_Angle = 0; // 角度制
        float Calibration_Motor_Up_Target_Omega_Radian = 0;  // 弧度制
        // 电机堵转校准阈值
        float Calibration_Motor_Yaw_Troque_Threshold = 0;
        float Calibration_Motor_Up_Torque_Threshold = 0;

        // 电机校准编码器角度
        float Calibration_Motor_Yaw_Angle_Offset = 0;  // 角度制
        float Calibration_Motor_Up_Radian_Offset = 0;  // 弧度制

        // 四个舵机装弹
        void Servo_Reload();
        // 四个舵机恢复初始位置
        void Servo_Init();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
