/**
 * @file dvc_supercap.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 超级电容
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿, 由于哨兵无超级电容, 因此未启用
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DVC_SUPERCAP_H
#define DVC_SUPERCAP_H
#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"
#include "drv_can.h"
#include "drv_uart.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 超级电容状态
 *
 */
enum Enum_Supercap_Status
{
    Supercap_Status_DISABLE = 0,
    Supercap_Status_ENABLE,
};
enum Enum_Supercap_Control_Status : uint8_t
{
    Supercap_Control_Status_ENABLE = 0,
    Supercap_Control_Status_DISABLE,
};
/**
 * @brief 超级电容源数据
 *Capacitor charge percentage
 */
struct Struct_Supercap_CAN_Data
{
    int16_t Chassis_Actual_Power;
    int16_t Supercap_Buffer_Power;
    uint8_t Supercap_Charge_Percentage;
    uint8_t Supercup_Control_Level_Status;
    uint16_t Supercap_Current_Energy_Consumption;
} __attribute__((packed));
/**
 * @brief 超级电容源处理后的数据
 *
 */
struct Struct_Supercap_Data
{
    float Chassis_Actual_Power;
    float Supercap_Buffer_Power;
    float Supercap_Charge_Percentage;
    uint8_t Supercup_Control_Level_Status;
    uint8_t Supercap_Current_Energy_Consumption;
}__attribute__((packed));
/**
 * @brief 超级电容发送的数据
 *
 */
struct Struct_Supercap_Tx_Data
{
    float Limit_Power;
    Enum_Supercap_Control_Status Supercap_Control_Status;
}__attribute__((packed));

/**
 * @brief Specialized, 超级电容
 * 
 */
class Class_Supercap
{
public:
    void Init(CAN_HandleTypeDef *__hcan, float __Limit_Power_Max = 45);
    void Init_UART(UART_HandleTypeDef *__huart, uint8_t __fame_header = '*', uint8_t __fame_tail = ';', float __Limit_Power_Max = 45.0f);

    inline Enum_Supercap_Status Get_Supercap_Status();

    inline void Set_Limit_Power(float __Limit_Power);
    inline void Set_Supercap_Control_Status(Enum_Supercap_Control_Status __Supercap_Control_Status);
    inline Enum_Supercap_Control_Status Get_Supercap_Control_Status();
    inline float Get_Chassis_Actual_Power();
    inline float Get_Limit_Power();
    inline float Get_Supercap_Buffer_Power();
    inline float Get_Supercap_Charge_Percentage();

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void UART_RxCpltCallback(uint8_t *Rx_Data);

    void TIM_Alive_PeriodElapsedCallback();

    void TIM_UART_Tx_PeriodElapsedCallback();
    void TIM_Supercap_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, 切记避开0x201~0x20b, 默认收包CAN1的0x210, 滤波器最后一个, 发包CAN1的0x220
    uint16_t CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;

    //串口模式
    Struct_UART_Manage_Object *UART_Manage_Object;
    uint8_t Fame_Header;
    uint8_t Fame_Tail;
    //常量

    //内部变量

    //当前时刻的超级电容接收flag
    uint32_t Flag = 0;
    //前一时刻的超级电容接收flag
    uint32_t Pre_Flag = 0;

    //读变量

    //超级电容状态
    Enum_Supercap_Status Supercap_Status = Supercap_Status_DISABLE;
    //超级电容对外接口信息
    Struct_Supercap_CAN_Data Supercap_Data;
    Struct_Supercap_Data Data;
    //写变量
    Struct_Supercap_Tx_Data Supercap_Tx_Data;

    //写变量

    //限制的功率
    float Limit_Power = 0.0f;

    //读写变量

    //内部函数

    void Data_Process();
    void Output();

    void Data_Process_UART();
    void Output_UART();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取超级电容状态
 * 
 * @return Enum_Supercap_Status 超级电容状态
 */
Enum_Supercap_Status Class_Supercap::Get_Supercap_Status()
{
    return(Supercap_Status);
}
/**
 * @brief 获取超级电容发送的缓冲功率
 * 
 * @return 超级电容发送的缓冲功率
 */
float Class_Supercap::Get_Supercap_Buffer_Power()
{
    return (Data.Supercap_Buffer_Power);
}
/**
 * @brief 获取存储的能量
 *
 * @return float 存储的能量
 */
// float Class_Supercap::Get_Stored_Energy()
// {
//     return (Supercap_Data.Stored_Energy);
// }

// /**
//  * @brief 获取输出的功率
//  *
//  * @return float 输出的功率
//  */
// float Class_Supercap::Get_Now_Power()
// {
//     return (Supercap_Data.Now_Power);
// }

/**
 * @brief 获取当前的电压
 *
 * @return float 当前的电压
 */
// float Class_Supercap::Get_Now_Voltage()
// {
//     return (Supercap_Data.Supercap_Voltage);
// }

/**
 * @brief 设置底盘当前的功率
 *
 * @return float 输入的功率
 */
// void Class_Supercap::Set_Now_Power(float __Now_Power)
// {
//     Supercap_Tx_Data.Now_Power = __Now_Power;
// }

/**
 * @brief 设定绝对最大限制功率
 *
 * @param __Limit_Power 绝对最大限制功率
 */
void Class_Supercap::Set_Limit_Power(float __Limit_Power)
{
    Supercap_Tx_Data.Limit_Power = __Limit_Power;
}
void Class_Supercap::Set_Supercap_Control_Status(Enum_Supercap_Control_Status __Supercap_Control_Status)
{
    Supercap_Tx_Data.Supercap_Control_Status = __Supercap_Control_Status;
}
Enum_Supercap_Control_Status Class_Supercap::Get_Supercap_Control_Status()
{
    return (Supercap_Tx_Data.Supercap_Control_Status);
}
float Class_Supercap::Get_Chassis_Actual_Power()
{
    return (Data.Chassis_Actual_Power);
}
float Class_Supercap::Get_Limit_Power()
{
    return (Limit_Power);
}
float Class_Supercap::Get_Supercap_Charge_Percentage()
{
    return (Data.Supercap_Charge_Percentage);
}
#endif
#endif
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
