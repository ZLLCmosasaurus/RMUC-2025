/**
 * @file dvc_dmmotor.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 瓴控电机配置与操作
 * @version 0.1
 * @date 2023-08-30 0.1 初稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DVC_LKMOTOR_H
#define DVC_LKMOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"
#include "drv_can.h"
#include "alg_pid.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 达妙电机状态
 *
 */
enum Enum_LK_Motor_Status
{
    LK_Motor_Status_DISABLE = 0,
    LK_Motor_Status_ENABLE,
};

/**
 * @brief 达妙电机的ID枚举类型
 *
 */
enum Enum_LK_Motor_ID : uint8_t
{
    LK_Motor_ID_UNDEFINED = 0,
    LK_Motor_ID_0x141,
    LK_Motor_ID_0x142,
    LK_Motor_ID_0x143,
    LK_Motor_ID_0x144,
    LK_Motor_ID_0x145,
    LK_Motor_ID_0x146,
    LK_Motor_ID_0x147,
    LK_Motor_ID_0x148,
};

/**
 * @brief 达妙电机控制状态
 *
 */
enum Enum_LK_Motor_Control_Status
{
    LK_Motor_Control_Status_DISABLE = 0,
    LK_Motor_Control_Status_ENABLE,
};

/**
 * @brief 达妙电机can控制cmd_id
 *
 */
enum Enum_LK_Motor_Control_ID : uint8_t
{
    LK_Motor_Control_Shut_Down = 0x80,   //电机关闭
    LK_Motor_Control_Stop = 0x81, //电机停止
    LK_Motor_Control_Run = 0x88,//电机运行
    LK_Motor_Control_Torque = 0xA1,//力矩闭环控制
};

/**
 * @brief 达妙电机控制方式
 *
 */
enum Enum_LK_Motor_Control_Method
{
    LK_Motor_Control_Method_IMU_ANGLE = 0,
    LK_Motor_Control_Method_IMU_OMEGA,
    LK_Motor_Control_Method_ANGLE,
    LK_Motor_Control_Method_OMEGA,
    LK_Motor_Control_Method_TORQUE,
};

/**
 * @brief 达妙电机源数据
 *
 */
struct Struct_LK_Motor_CAN_Rx_Data
{
    Enum_LK_Motor_Control_ID CMD_ID;
    uint8_t Temperature_Centigrade;  //摄氏度
    uint16_t Current_Reverse;
    uint16_t Omega_Reverse;
    uint16_t Encoder_Reverse;
} __attribute__((packed));

/**
 * @brief 达妙电机经过处理的数据, 扭矩非国际单位制
 *
 */
struct Struct_LK_Motor_Rx_Data
{
    Enum_LK_Motor_Control_ID CMD_ID;  //控制命令ID
    float Now_Angle;  //角度制
    float Now_Omega;  //弧度制
    float Now_Current;  //安培
    float Now_Temperature; //摄氏度
    uint16_t Pre_Encoder; 
    int32_t Total_Encoder;
    int32_t Total_Round;
};

/**
 * @brief LK无刷电机, 单片机控制输出控制帧
 * DM_Motor_Control_Method_POSITION_OMEGA模式下, 需调参助手辅助设置位置环PI参数, 空载250与0
 * 
 * PMAX值需在调参助手设置为3.141593, 即PI, 此时可在IMU模式下当舵机使用
 *
 */
class Class_LK_Motor
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;
    // PID扭矩环控制
    Class_PID PID_Torque;

    void Init(CAN_HandleTypeDef *hcan, Enum_LK_Motor_ID __CAN_ID, float __Omega_Max, int32_t __Position_Offset = 0, float __Current_Max = 33.0f ,Enum_LK_Motor_Control_Method __Control_Method = LK_Motor_Control_Method_IMU_ANGLE);

    inline Enum_LK_Motor_Control_Status Get_LK_Motor_Control_Status();
    inline Enum_LK_Motor_Status Get_LK_Motor_Status();
    inline float Get_Output_Max();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline float Get_Now_Temperature();
    inline Enum_LK_Motor_Control_Method Get_Control_Method();
    inline float Get_IMU_K_P();
    inline float Get_IMU_K_D();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();
    
    inline void Set_LK_Control_Status(Enum_LK_Motor_Control_Status __DM_Motor_Control_Status);
    inline void Set_LK_Motor_Control_Method(Enum_LK_Motor_Control_Method __DM_Motor_Control_Method);
    inline void Set_IMU_K_P(float __IMU_K_P);
    inline void Set_IMU_K_D(float __IMU_K_D);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Current(float __Target_Current);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_Out(float __Out);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Process_PeriodElapsedCallback();

protected:
    //初始化相关变量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, 控制帧是0xxa1~0xxaf
    Enum_LK_Motor_ID CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;
    //位置反馈偏移
    uint32_t Position_Offset;
    //最大速度, 调参助手设置, 推荐20.94359, 也就是最大转速200rpm
    float Omega_Max;
    //最大扭矩, 调参助手设置, 推荐7, 也就是最大输出7NM
    float Current_Max;
    //最大发送电流值
    const int16_t Current_Max_Cmd = 2000;
    //最终输出量
    float Out = 0.0f;
    //常量
    
    const float Torque_Current = 0.3;  //瞎写的 电流转矩系数

    //一圈位置刻度
    uint32_t Position_Max = 16383;


    
    //内部变量

    //当前时刻的电机接收flag
    uint32_t Flag = 0;
    //前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;

    //读变量

    //电机状态
    Enum_LK_Motor_Status LK_Motor_Status = LK_Motor_Status_DISABLE;
    //电机对外接口信息
    Struct_LK_Motor_Rx_Data Data;

    //写变量

    //读写变量

    //电机电驱控制模式
    Enum_LK_Motor_Control_ID LK_Motor_Control_ID = LK_Motor_Control_Torque;
    //电机控制状态
    Enum_LK_Motor_Control_Status LK_Motor_Control_Status = LK_Motor_Control_Status_DISABLE;
    //电机控制方式
    Enum_LK_Motor_Control_Method LK_Motor_Control_Method = LK_Motor_Control_Method_IMU_ANGLE;
    //MIT的Kp值, 0~500, 空载6, 位置控制需要
    float IMU_K_P = 0.0f;
    //MIT的Kd值, 0~5, 空载0.2, 位置和速度控制需要
    float IMU_K_D = 0.0f;
    //目标的角度, rad
    float Target_Angle = 0.0f;
    //目标的速度, rad/s
    float Target_Omega = 0.0f;
    //目标的电流
    float Target_Current = 0.0f;
    //目标力矩
    float Target_Torque = 0.0f;
    //内部函数
 
    void Output(void);
    void Data_Process();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return Enum_LK_Motor_Status 电机状态
 */
Enum_LK_Motor_Status Class_LK_Motor::Get_LK_Motor_Status()
{
    return (LK_Motor_Status);
}


/**
 * @brief 获取电机状态
 *
 * @return Enum_LK_Motor_Status 电机状态
 */
Enum_LK_Motor_Control_Status Class_LK_Motor::Get_LK_Motor_Control_Status()
{
    return (LK_Motor_Control_Status);
}


/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_LK_Motor::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
float Class_LK_Motor::Get_Now_Omega()
{
    return (Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, 直接采用反馈值
 *
 * @return float 当前的扭矩, 直接采用反馈值
 */
float Class_LK_Motor::Get_Now_Torque()
{
    return (Data.Now_Current);
}

/**
 * @brief 获取当前MOS管的温度, 开氏度
 *
 * @return float 当前MOS管的温度, 开氏度
 */
float Class_LK_Motor::Get_Now_Temperature()
{
    return (Data.Now_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_LK_Motor_Control_Method 电机控制方式
 */
Enum_LK_Motor_Control_Method Class_LK_Motor::Get_Control_Method()
{
    return (LK_Motor_Control_Method);
}

/**
 * @brief 获取MIT的Kp值, 0~500
 *
 * @return float MIT的Kp值, 0~500
 */
float Class_LK_Motor::Get_IMU_K_P()
{
    return (IMU_K_P);
}

/**
 * @brief 获取MIT的Kd值, 0~5
 *
 * @return float MIT的Kd值, 0~5
 */
float Class_LK_Motor::Get_IMU_K_D()
{
    return (IMU_K_D);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
float Class_LK_Motor::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
float Class_LK_Motor::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩
 *
 * @return float 目标的扭矩
 */
float Class_LK_Motor::Get_Target_Torque()
{
    return (Target_Current);
}

/**
 * @brief 设定电机控制状态
 *
 * @param __DM_Motor_Control_Status 电机控制状态
 */
void Class_LK_Motor::Set_LK_Control_Status(Enum_LK_Motor_Control_Status __DM_Motor_Control_Status)
{
    LK_Motor_Control_Status = __DM_Motor_Control_Status;
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Control_Method 电机控制方式
 */
void Class_LK_Motor::Set_LK_Motor_Control_Method(Enum_LK_Motor_Control_Method __Control_Method)
{
    LK_Motor_Control_Method = __Control_Method;
}

/**
 * @brief 设定MIT的Kp值, 0~500, 空载6, 位置控制需要
 *
 * @param __MIT_K_P MIT的Kp值, 0~500, 空载6, 位置控制需要
 */
void Class_LK_Motor::Set_IMU_K_P(float __IMU_K_P)
{
    IMU_K_P = __IMU_K_P;
}

/**
 * @brief 设定MIT的Kd值, 0~5, 空载0.2, 位置和速度控制需要
 *
 * @param __MIT_K_D MIT的Kd值, 0~5, 空载0.2, 位置和速度控制需要
 */
void Class_LK_Motor::Set_IMU_K_D(float __IMU_K_D)
{
    IMU_K_D = __IMU_K_D;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
void Class_LK_Motor::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
void Class_LK_Motor::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的力矩电流
 *
 * @param __Target_Current 目标的力矩电流
 */
void Class_LK_Motor::Set_Target_Current(float __Target_Current)
{
    Target_Current = __Target_Current;
}

/**
 * @brief 设定目标的力矩电流
 *
 * @param __Target_Torque 目标的力矩电流
 */
void Class_LK_Motor::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

void Class_LK_Motor::Set_Out(float __Out)
{
    if(__Out > Current_Max_Cmd)
    {
        __Out = Current_Max_Cmd;
    }
    else if(__Out < -Current_Max_Cmd)
    {
        __Out = -Current_Max_Cmd;
    }
}

float Class_LK_Motor::Get_Output_Max()
{
    return (Current_Max_Cmd);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
