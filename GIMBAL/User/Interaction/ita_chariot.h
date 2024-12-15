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
#include "crt_chassis.h"
#include "crt_gimbal.h"
#include "crt_booster.h"
#include "dvc_imu.h"
#include "tsk_config_and_callback.h"
#include "dvc_supercap.h"

/* Exported macros -----------------------------------------------------------*/

//#define CHASSIS
#define GIMBAL
#define AGV
//#define POWER_LIMIT

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 控制对象
 *
 */
class Class_Chariot
{
public:
    #ifdef CHASSIS
    //裁判系统

        #ifdef POWER_LIMIT
        //超级电容
        Class_Supercap Supercap;

        // 底盘随动PID环
        Class_PID PID_Chassis_Fllow;

        // 获取yaw电机编码器值 用于底盘和云台坐标系的转换
        Class_DJI_Motor_GM6020 Motor_Yaw;
        #endif

    #endif
    //底盘
    Class_Tricycle_Chassis Chassis;

    Class_Referee Referee;
#ifdef GIMBAL
    //遥控器
    Class_DR16 DR16;
    //上位机
    Class_MiniPC MiniPC;
    //云台
    Class_Gimbal Gimbal;
    //发射机构
    Class_Booster Booster;
#endif

    void Init(float __DR16_Dead_Zone = 0);
    
    #ifdef CHASSIS
    void CAN_Chassis_Control_RxCpltCallback();

    #elif defined(GIMBAL)
    void CAN_Gimbal_RxCpltCallback();
    void CAN_Gimbal_TxCpltCallback();
    void TIM_Control_Callback();
#endif

    void TIM_Calculate_PeriodElapsedCallback();
    void TIM1msMod50_Alive_PeriodElapsedCallback();
    
protected:
    //初始化相关常量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object = &CAN2_Manage_Object;
    //发送缓存区
    uint8_t *CAN_Tx_Data = CAN2_Gimbal_Tx_Data;
    uint8_t *CAN_Rx_Data = CAN2_Chassis_Tx_Data;
    //遥控器拨动的死区, 0~1
    float DR16_Dead_Zone;

    //常量
    //底盘标定参考正方向角度(数据来源yaw电机)
    float Reference_Angle = 2.23402;
    //底盘转换后的角度（数据来源yaw电机）
    float Chassis_Angle;

    //DR16底盘加速灵敏度系数(0.001表示底盘加速度最大为1m/s2)
    float DR16_Keyboard_Chassis_Speed_Resolution_Small = 0.001f;
    //DR16底盘减速灵敏度系数(0.001表示底盘加速度最大为1m/s2)
    float DR16_Keyboard_Chassis_Speed_Resolution_Big = 0.01f;

    //DR16云台yaw灵敏度系数(0.001PI表示yaw速度最大时为1rad/s)
    float DR16_Yaw_Angle_Resolution = 0.005f * PI * 57.29577951308232;
    //DR16云台pitch灵敏度系数(0.001PI表示pitch速度最大时为1rad/s)
    float DR16_Pitch_Angle_Resolution = 0.0035f * PI * 57.29577951308232;

    //DR16云台yaw灵敏度系数(0.001PI表示yaw速度最大时为1rad/s)
    float DR16_Yaw_Resolution = 0.003f * PI;
    //DR16云台pitch灵敏度系数(0.001PI表示pitch速度最大时为1rad/s)
    float DR16_Pitch_Resolution = 0.003f * 120;

    //DR16鼠标云台yaw灵敏度系数, 不同鼠标不同参数
    float DR16_Mouse_Yaw_Angle_Resolution = 500.0f;
    //DR16鼠标云台pitch灵敏度系数, 不同鼠标不同参数
    float DR16_Mouse_Pitch_Angle_Resolution = 500.0f;
    
    //迷你主机云台pitch自瞄控制系数
    float MiniPC_Autoaiming_Yaw_Angle_Resolution = 0.003f;
    //迷你主机云台pitch自瞄控制系数
    float MiniPC_Autoaiming_Pitch_Angle_Resolution = 0.003f;

    //内部变量

    //读变量

    //写变量

    //读写变量

    //内部函数

    void Control_Chassis();
    void Control_Gimbal();
    void Control_Booster();

};

#ifdef AGV
typedef __packed enum {
    CHASSIS_MODE_NOFORCE = 0x00u,
    CHASSIS_MODE_TOPANGLE = 0x02u,
    CHASSIS_MODE_ABSOLUTE = 0x01u,
    CHASSIS_MODE_PRECISE = 0x03u,
} CHASSIS_MODE_E;

typedef __packed struct
{
    float vx;
    float vy;
    float vw;
} CHASSIS_VELOCITY_T;

typedef __packed struct
{
    CHASSIS_MODE_E mode;
    CHASSIS_VELOCITY_T velocity;
    bool follow_flag;
    bool invert_flag;

} CHASSIS_SEND_T;

typedef __packed struct
{
    CHASSIS_MODE_E mode;
    CHASSIS_VELOCITY_T velocity;

} CHASSIS_RECEIVE_T;

typedef __packed struct
{
    CHASSIS_SEND_T send;
    CHASSIS_RECEIVE_T receive;

} CHASSIS_T;
void Chassis_Connection_Init(void);
extern CHASSIS_T chassis;
void Chassis_Connection_Task(void);
void Can_Send_Task(int8_t ms_count);
void Can_Connection_Init(void);
#endif

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
