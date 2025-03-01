/**
 * @file dvc_AKmotor.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief AK电机配置与操作
 * @version 0.1
 * @date 2023-08-30 0.1 初稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "drv_can.h"
#include "alg_pid.h"
#include "alg_slope.h"
#include "drv_math.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

extern uint8_t AK_Motor_CAN_Message_Enter[8];

#define RAD_TO_DEG 57.29578f

#define AK80_DUTY_CYCLE_FROM_FLOAT_TO_LSB(duty_cycle) (uint32_t)(duty_cycle * 10000.0f)
#define AK80_CURRENT_FROM_FLOAT_TO_LSB(current) (int32_t)(current * 1000.0f)
#define AK80_BARKING_CURRENT_FROM_FLOAT_TO_LSB(barking_current) (int32_t)(barking_current * 1000.0f)
#define AK80_SPEED_FROM_FLOAT_TO_LSB(speed) (int32_t)(speed * 1.0f)
#define AK80_POSITION_FROM_FLOAT_TO_LSB(position) (int32_t)(position * 10000.0f)
#define AK80_SPEED_POSITION_SPD_FROM_FLOAT_TO_LSB(speed) (int16_t)(speed / 10.0f)
#define AK80_SPEED_POSITION_ACL_FROM_FLOAT_TO_LSB(acceleration) (int16_t)(acceleration / 10.0f)

#define AK80_POSITION_FROM_LSB_TO_FLOAT(position) (float)((int16_t)position / 10.0f)
#define AK80_SPEED_FROM_LSB_TO_FLOAT(speed) (float)((int16_t)speed * 10.0f)
#define AK80_CURRENT_FROM_LSB_TO_FLOAT(current) (float)((int16_t)current / 100.0f)
#define AK80_TEMPERATURE_FROM_LSB_TO_FLOAT(temperature) (float)((int8_t)temperature * 1.0f)

#define AK80_RUN_CONTROL_POSITION_FROM_FLOAT_TO_UINT(position) (uint16_t)((float)position / 12.5f * 32767 + 32767)
#define AK80_RUN_CONTROL_SPEED_FROM_FLOAT_TO_UINT(speed) (uint16_t)((float)speed / 76.0f * 2047 + 2047)
#define AK80_RUN_CONTROL_TORQUE_FROM_FLOAT_TO_UINT(torque) (uint16_t)((float)torque / 12.0f * 2047 + 2047)
#define AK80_RUN_CONTROL_KP_FROM_FLOAT_TO_UINT(kp) (uint16_t)((float)kp / 500.0f * 4095.0f)
#define AK80_RUN_CONTROL_KD_FROM_FLOAT_TO_UINT(kd) (uint16_t)((float)kd / 5.0f * 4095.0f)

#define AK80_RUN_CONTROL_POSITION_FROM_UINT_TO_FLOAT(position) (float)(((int)position - 32767) / 32767 * 76.0f)
#define AK80_RUN_CONTROL_SPEED_FROM_UINT_TO_FLOAT(speed) (float)(((int)speed - 2047) / 2047 * 76.0f)
#define AK80_RUN_CONTROL_TORQUE_FROM_UINT_TO_FLOAT(torque) (float)(((int)torque - 2047) / 2047 * 12.0f)
#define AK80_RUN_CONTROL_TEMPERATURE_FROM_UINT_TO_FLOAT(temperature) (float)((int)temperature)
/**
 * @brief AK电机状态
 *
 */
enum Enum_AK_Motor_Status
{
    AK_Motor_Status_DISABLE = 0,
    AK_Motor_Status_ENABLE,
};

/**
 * @brief AK电机的ID枚举类型
 *
 */
enum Enum_AK_Motor_ID : uint8_t
{
    AK_Motor_ID_0x01 = 0x11,
    AK_Motor_ID_0x02,
    AK_Motor_ID_0x03,
    AK_Motor_ID_0x04,
    AK_Motor_ID_0x05,
    AK_Motor_ID_0x06,
    AK_Motor_ID_0x07,
    AK_Motor_ID_0x08,

};

/**
 * @brief AK电机的错误码类型
 *
 */
enum ERROR_STATUE_TYPE_T : uint8_t
{
    NONE_ERROR = 0,
    OVER_TEMPERATURE_ERROR,
    OVER_CURRENT_ERROR,
    OVER_VOLTAGE_ERROR,
    UNDER_VOLTAGE_ERROR,
    ENCODER_ERROR,
    CURRENT_UNBALANCE_ERROR,
};
/**
 * @brief AK电机控制状态
 *
 */
enum Enum_AK_Motor_Control_Status
{
    AK_Motor_Control_Status_DISABLE = 0,
    AK_Motor_Control_Status_ENABLE,
};

/**
 * @brief AK电机控制方式,设置零点不可使用，无效
 *
 */
enum Enum_AK_Motor_Control_Method
{
    CAN_PACKET_SET_DUTY,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_ORIGIN_HERE,
    CAN_PACKET_SET_POS_SPD,
    CAN_PACKET_SET_RUN_CONTROL,
    CAN_PACKET_DIS_RUN_CONTROL,
};

/**
 * @brief AK电机伺服源数据
 *
 */
struct Struct_AK_Motor_CAN_Rx_Data
{
    uint16_t Position_Reverse;
    uint16_t Omega_Reverse;
    uint16_t Torque_Reverse;
    uint8_t Motor_Temperature;
    ERROR_STATUE_TYPE_T error_statue;
} __attribute__((packed));

/**
 * @brief AK电机运控源数据
 *
 */
struct Struct_AK_Motor_RUN_CAN_Rx_Data
{
    Enum_AK_Motor_ID CAN_ID;
    uint16_t Position_Reverse;
    uint8_t Omega_11_4;
    uint8_t Omega_3_0_Torque_11_8;
    uint8_t Torque_7_0;
    uint8_t Motor_Temperature;
    ERROR_STATUE_TYPE_T error_statue;
} __attribute__((packed));
/**
 * @brief AK电机经过处理的数据, 扭矩非国际单位制
 *
 */
struct Struct_AK_Motor_Rx_Data
{
    Enum_AK_Motor_ID CAN_ID;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_Rotor_Temperature;
    ERROR_STATUE_TYPE_T error_statue;
    uint16_t Pre_Position;
    int32_t Total_Position;
    int32_t Total_Round;

    bool operator!=(const uint8_t &a)
    {
        if ((this->Now_Angle != a) || (this->Now_Omega != a) || (this->Now_Torque != a))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void operator=(const uint8_t &a)
    {
        this->CAN_ID = static_cast<Enum_AK_Motor_ID>(a);
        this->Now_Angle = static_cast<float>(a);
        this->Now_Omega = static_cast<float>(a);
        this->Now_Torque = static_cast<float>(a);
        this->Now_Rotor_Temperature = static_cast<float>(a);
        this->error_statue = static_cast<ERROR_STATUE_TYPE_T>(a);
        this->Pre_Position = static_cast<uint16_t>(a);
        this->Total_Position = static_cast<int32_t>(a);
        this->Total_Round = static_cast<int32_t>(a);
    }
};

/**
 * @brief AK无刷电机, 单片机控制输出控制帧
 * AK_Motor_Control_Method_POSITION_OMEGA模式下, 需调参助手辅助设置位置环PI参数, 空载250与0
 *
 * PMAX值需在调参助手设置为3.141593, 即PI, 此时可在MIT模式下当舵机使用
 *
 */
class Class_AK_Motor_80_6
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // 斜坡函数加减速速度X
    Class_Slope Slope_Joint_Angle;
    void Init(CAN_HandleTypeDef *hcan, Enum_AK_Motor_ID __CAN_ID, Enum_AK_Motor_Control_Method __Control_Method = CAN_PACKET_SET_CURRENT, float __MIT_K_P = 12.0f, float __MIT_K_D = 0.8f,
              int32_t __Position_Offset = 0, float __Angle_Max = 12.5f, float __Omega_Max = 76.0f, float __Torque_Max = 12.0f, float __Slope_Angle = 0.1f);

    inline Enum_AK_Motor_Control_Status Get_AK_Motor_Control_Status();
    inline Enum_AK_Motor_Status Get_AK_Motor_Status();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline Struct_AK_Motor_Rx_Data Get_Rx_Data();
    inline float Get_Now_Rotor_Temperature();
    inline Enum_AK_Motor_Control_Method Get_Control_Method();
    inline float Get_MIT_K_P();
    inline float Get_MIT_K_D();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();

    inline void Set_AK_Control_Status(Enum_AK_Motor_Control_Status __AK_Motor_Control_Status);
    inline void Set_AK_Motor_Control_Method(Enum_AK_Motor_Control_Method __AK_Motor_Control_Method);
    inline void Set_MIT_K_P(float __MIT_K_P);
    inline void Set_MIT_K_D(float __MIT_K_D);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_Target_Current(float __Target_Current);
    inline void Reset_Rx_Data();
    inline float Get_KT();
    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void Task_Alive_PeriodElapsedCallback();
    void Task_PID_PeriodElapsedCallback();
    void Task_Process_PeriodElapsedCallback();

    float Target_Current = 0.0f;

protected:
    // 初始化相关变量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, 控制帧是0xxa1~0xxaf
    Enum_AK_Motor_ID CAN_ID;
    // 发送缓存区
    uint8_t *CAN_Tx_Data;
    // 位置反馈偏移
    uint32_t Position_Offset;
    // 最大扭矩, 调参助手设置, 推荐7, 也就是最大输出7NM
    float Angle_Max;
    // 最大速度, 调参助手设置, 推荐20.94359, 也就是最大转速200rpm
    float Omega_Max;
    // 最大扭矩, 调参助手设置, 推荐7, 也就是最大输出7NM
    float Torque_Max;

    // 常量

    // 一圈位置刻度
    uint32_t Position_Max = 36000;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;

    // 读变量
    float KT = 0.09549f;
    // 电机状态
    Enum_AK_Motor_Status AK_Motor_Status = AK_Motor_Status_DISABLE;

    // 写变量

    // 读写变量

    // 电机对外接口信息
    Struct_AK_Motor_Rx_Data Data;
    // 电机控制状态
    Enum_AK_Motor_Control_Status AK_Motor_Control_Status = AK_Motor_Control_Status_DISABLE;
    // 电机控制方式
    Enum_AK_Motor_Control_Method AK_Motor_Control_Method = CAN_PACKET_SET_CURRENT;
    // MIT的Kp值, 0~500, 空载6, 位置控制需要
    float MIT_K_P;
    // MIT的Kd值, 0~5, 空载0.2, 位置和速度控制需要
    float MIT_K_D;
    // 目标的角度, rad
    float Target_Angle = 0.0f;
    // 目标的速度, rad/s
    float Target_Omega = 0.0f;
    // 目标的扭矩
    float Target_Torque = 0.0f;

    // 内部函数

    void Data_Process();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return Enum_AK_Motor_Status 电机状态
 */
Enum_AK_Motor_Status Class_AK_Motor_80_6::Get_AK_Motor_Status()
{
    return (AK_Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_AK_Motor_80_6::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
float Class_AK_Motor_80_6::Get_Now_Omega()
{
    return (Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, 直接采用反馈值
 *
 * @return float 当前的扭矩, 直接采用反馈值
 */
float Class_AK_Motor_80_6::Get_Now_Torque()
{
    return (Data.Now_Torque);
}

/**
 * @brief 获取当前的接受数据, 直接采用反馈值
 *
 * @return Struct_AK_Motor_Rx_Data 当前的接收数据, 直接采用反馈值
 */
Struct_AK_Motor_Rx_Data Class_AK_Motor_80_6::Get_Rx_Data()
{
    return Data;
}
/**
 * @brief 获取当前绕组的温度, 开氏度
 *
 * @return float 当前绕组的温度, 开氏度
 */
float Class_AK_Motor_80_6::Get_Now_Rotor_Temperature()
{
    return (Data.Now_Rotor_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_AK_Motor_Control_Method 电机控制方式
 */
Enum_AK_Motor_Control_Method Class_AK_Motor_80_6::Get_Control_Method()
{
    return (AK_Motor_Control_Method);
}

/**
 * @brief 获取MIT的Kp值, 0~500
 *
 * @return float MIT的Kp值, 0~500
 */
float Class_AK_Motor_80_6::Get_MIT_K_P()
{
    return (MIT_K_P);
}

/**
 * @brief 获取MIT的Kd值, 0~5
 *
 * @return float MIT的Kd值, 0~5
 */
float Class_AK_Motor_80_6::Get_MIT_K_D()
{
    return (MIT_K_D);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
float Class_AK_Motor_80_6::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
float Class_AK_Motor_80_6::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩
 *
 * @return float 目标的扭矩
 */
float Class_AK_Motor_80_6::Get_Target_Torque()
{
    return (Target_Torque);
}

/**
 * @brief 设定电机控制状态
 *
 * @param __AK_Motor_Control_Status 电机控制状态
 */
void Class_AK_Motor_80_6::Set_AK_Control_Status(Enum_AK_Motor_Control_Status __AK_Motor_Control_Status)
{
    AK_Motor_Control_Status = __AK_Motor_Control_Status;
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Control_Method 电机控制方式
 */
void Class_AK_Motor_80_6::Set_AK_Motor_Control_Method(Enum_AK_Motor_Control_Method __Control_Method)
{
    AK_Motor_Control_Method = __Control_Method;
}

/**
 * @brief 设定MIT的Kp值, 0~500, 空载6, 位置控制需要
 *
 * @param __MIT_K_P MIT的Kp值, 0~500, 空载6, 位置控制需要
 */
void Class_AK_Motor_80_6::Set_MIT_K_P(float __MIT_K_P)
{
    MIT_K_P = __MIT_K_P;
}

/**
 * @brief 设定MIT的Kd值, 0~5, 空载0.2, 位置和速度控制需要
 *
 * @param __MIT_K_D MIT的Kd值, 0~5, 空载0.2, 位置和速度控制需要
 */
void Class_AK_Motor_80_6::Set_MIT_K_D(float __MIT_K_D)
{
    MIT_K_D = __MIT_K_D;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
void Class_AK_Motor_80_6::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
void Class_AK_Motor_80_6::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

void Class_AK_Motor_80_6::Reset_Rx_Data()
{
    Data = 0;
}
/**
 * @brief 设定目标的扭矩
 *
 * @param __Target_Torque 目标的扭矩
 */
void Class_AK_Motor_80_6::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

void Class_AK_Motor_80_6::Set_Target_Current(float __Target_Current)
{
    Target_Current = __Target_Current;
}
float Class_AK_Motor_80_6::Get_KT(void)
{
    return KT;
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
