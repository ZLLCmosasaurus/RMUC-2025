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

    bool Tension_Status = false;            //记录传动电机是否完成传动，防止重复操作（因为按键按下后又会松开）
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

        //测试拉力闭环时使用
        void Test_Tension();
        //
        void Test_PID();
        //测试左右传动电机
        void Test_Motor_LR();

        void Updata_Motor_Left_Right_Offset();
        void Updata_Distance_Angle();
        
        bool Calibrate();
        
        inline void Updata_Switch_Status();

        inline float Distance_to_Radian_LR(float Target_Distance, float Zero_Position);
        inline float Get_Calibration_Position_Left();
        inline float Get_Calibration_Position_Right();

        inline void Updata_Calibration_Position_Left();
        inline void Updata_Calibration_Position_Right();

        void TIM_Control_Callback();
        void TIM_Calculate_PeriodElapsedCallback();
        void TIM5msMod10_Alive_PeriodElapsedCallback();

        void Motor_Up_Target_Distance_Updata();
        void Motor_Up_Speed_Updata();

        void Servo_Lock();
        void Servo_Unlock();

        //迷你主机状态
        Enum_MiniPC_Status MiniPC_Status = MiniPC_Status_DISABLE;

protected:

        //每次拨弹前进距离
        float Add_Distance =  100.0f;

        //遥控器拨动的死区, 0~1
        float DR16_Dead_Zone;
        //DR16云台yaw灵敏度系数(0.001PI表示yaw速度最大时为1rad/s)
        float DR16_Yaw_Angle_Resolution = 0.005f * PI * 57.29577951308232;
        //内部变量

        // 推镖电机 丝杆比例系数 mm
        float Radian_To_Distance_Motor_Up = 4.0f;
        //拉力电机比例系数
        float Radian_To_Distance_Motor_Down = 4.0f;

        //Motor_Left R的距离比例系数
        float Radian_To_Distance_Motor_LR = 108.0f;

        // yaw电机减速比系数 没确定
        float Reduction_Ratio = 1.0f; 
        // 扳机舵机角度
        float Shoot_Angle_Trigger; //按下角度
        float Close_Angle_Trigger; //松开角度

        // 推镖电机distance
        float Now_Distance_Motor_Up;   
        float Tartget_Distance_Motor_Up;
        float Target_Speed_Motor_Up = 8.0f;

        //右传动电机当前距离
        float Now_Distance_Motor_Right = 0.0f;

        //拉力电机当前距离
        float Now_Distance_Motor_Down = 0.0f;

        //微动开关状态，初始低电平false
        bool Switch_Bool = false;               

        //左右传动电机当前位置，最上端为0
        float Now_Distance_Motor_Left = 0.0f;

        //扣动扳机所需的目标距离
        float Target_Distance_Motor_LR = 1040.0f;
        
        //拉力电机目标拉力距离
        float Target_Distance_Motor_Down = 0.0f;

        //Motor_Left和Right的转速偏差
        float Motor_LR_Offset = 0.0f;

        // Yaw电机角度
        float Now_Angle_Yaw;
        float Target_Angle_Yaw;

        // 拉皮筋速度
        //Left 给正值向下，Right相反
        float Target_Speed_Motor_Left = 10.0f;
        float Target_Speed_Motor_Right = -10.0f;

        // 目标拉力
        float Target_Tension;

        //读变量
        
        // 校准完成标志位
        bool Calibration_Finish = false;

        
        bool Calibration_Motor_LR = false;

        // 电机校准目标角速度
        float Calibration_Motor_Yaw_Target_Omega_Angle = 0; // 角度制
        float Calibration_Motor_Up_Target_Omega_Radian = -10.0f; // 弧度制
        float Calibration_Motor_Down_Target_Omega_Radian = 10.0f;       //向上运动

        // 电机堵转校准阈值
        float Calibration_Motor_Yaw_Troque_Threshold = 0;
        float Calibration_Motor_Up_Torque_Threshold = 800;

        // 电机校准编码器角度
        float Calibration_Motor_Yaw_Angle_Offset = 0;  // 角度制
        float Calibration_Motor_Up_Radian_Offset = 0;  // 弧度制
        float Calibration_Motor_Left_Radian_Offset = 0;
        float Calibration_Motor_Right_Radian_Offset = 0;
        float Calibration_Motor_Down_Radian_Offset = 0;


        // 四个舵机装弹
        void Servo_Reload();
        // 四个舵机恢复初始位置
        void Servo_Init();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 传动电机距离环，距离转转动角度
 * @param Target_Distance 目标距离
 * @param Zero_Position 控制电机的零位
 */
float Class_Chariot::Distance_to_Radian_LR(float Target_Distance, float Zero_Position){
    return Target_Distance * (2 * PI)/ Radian_To_Distance_Motor_LR + Zero_Position;
}

/**
 * @brief 零位传出接口，方便调试
 */
float Class_Chariot::Get_Calibration_Position_Left(){
    return Calibration_Motor_Left_Radian_Offset;
}

/**
 * @brief 零位传出接口，方便调试
 */
float Class_Chariot::Get_Calibration_Position_Right(){
    return Calibration_Motor_Right_Radian_Offset;
}

/**
 * @brief 左传动电机零位校准
 */
void Class_Chariot::Updata_Calibration_Position_Left(){
    Calibration_Motor_Left_Radian_Offset = Motor_Left.Get_Now_Radian();
}

/**
 * @brief 右传动电机零位校准
 */
void Class_Chariot::Updata_Calibration_Position_Right(){
    Calibration_Motor_Right_Radian_Offset = Motor_Right.Get_Now_Radian();
}

/**
 * @brief 更新微动开关状态
 */
void Class_Chariot::Updata_Switch_Status(){
    Switch_Bool = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
