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

/**
 * @brief 超级电容源数据
 *
 */
struct Struct_Supercap_CAN_Data
{
    int16_t Stored_Energy_Reverse;
    int16_t Now_Power_Reverse;
} __attribute__((packed));

/**
 * @brief 超级电容经过处理的数据
 *
 */
struct Struct_Supercap_Data
{
    float Stored_Energy;
    float Now_Power;
};

/**
 * @brief Specialized, 超级电容
 * 
 */
class Class_Supercap
{
public:
    void Init(CAN_HandleTypeDef *__hcan, uint16_t __CAN_ID = 0x210, float __Limit_Power_Max = 0);
    void Init_UART(UART_HandleTypeDef *__huart, uint8_t __fame_header = '*', uint8_t __fame_tail = ';', float __Limit_Power_Max = 45.0f);

    inline Enum_Supercap_Status Get_Supercap_Status();
    inline float Get_Stored_Energy();
    inline float Get_Now_Power();

    inline void Set_Limit_Power(float __Limit_Power);
    inline void Set_Now_Power(float __Now_Power);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void UART_RxCpltCallback(uint8_t *Rx_Data);

    void TIM_Alive_PeriodElapsedCallback();
    void TIM_UART_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, 切记避开0x201~0x20b, 默认收包CAN1的0x210, 滤波器最后一个, 发包CAN1的0x220
    uint16_t CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;
    //绝对最大限制功率, 0表示不限制
    float Limit_Power_Max;

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
    Struct_Supercap_Data Supercap_Data;

    //写变量

    //限制的功率
    float Limit_Power = 0.0f;

    //读写变量

    //内部函数

    void Data_Process();
    void Data_Process_UART();

    void Output();
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
 * @brief 获取存储的能量
 *
 * @return float 存储的能量
 */
float Class_Supercap::Get_Stored_Energy()
{
    return (Supercap_Data.Stored_Energy);
}

/**
 * @brief 获取输出的功率
 *
 * @return float 输出的功率
 */
float Class_Supercap::Get_Now_Power()
{
    return (Supercap_Data.Now_Power);
}
/**
 * @brief 设置底盘当前的功率
 *
 * @return float 输入的功率
 */
void Class_Supercap::Set_Now_Power(float __Now_Power)
{
    Supercap_Data.Now_Power = __Now_Power;
}

/**
 * @brief 设定绝对最大限制功率
 *
 * @param __Limit_Power 绝对最大限制功率
 */
void Class_Supercap::Set_Limit_Power(float __Limit_Power)
{
    Limit_Power = __Limit_Power;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
