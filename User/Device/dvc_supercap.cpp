/**
 * @file dvc_supercap.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 超级电容
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "dvc_supercap.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化超级电容通信, 切记__CAN_ID避开0x201~0x20b, 默认收包CAN1的0x210, 滤波器最后一个, 发包CAN1的0x220
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_ID 收数据绑定的CAN ID
 * @param __Limit_Power_Max 最大限制功率, 0表示不限制
 */
void Class_Supercap::Init(CAN_HandleTypeDef *hcan, float __Limit_Power_Max)
{
    if(hcan->Instance == CAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hcan->Instance == CAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    Supercap_Tx_Data.Limit_Power = __Limit_Power_Max;
    CAN_Tx_Data = CAN_Supercap_Tx_Data;
}
/**
 * @brief 初始化超级电容通信
 *
 * @param __huart 绑定的CAN总线
 * @param __fame_header 收数据绑定的帧头
 * @param __fame_tail 收数据绑定的帧尾
 * @param __Limit_Power_Max 最大限制功率, 0表示不限制
 */
void Class_Supercap::Init_UART(UART_HandleTypeDef *__huart, uint8_t __fame_header, uint8_t __fame_tail, float __Limit_Power_Max )
{
    if (__huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (__huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (__huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (__huart->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (__huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (__huart->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }
    Supercap_Status = Supercap_Status_DISABLE;
    Supercap_Tx_Data.Limit_Power = __Limit_Power_Max;
    UART_Manage_Object->UART_Handler = __huart;
    Fame_Header = __fame_header;
    Fame_Tail = __fame_tail;
}

/**
 * @brief 
 * 
 */
void Class_Supercap::Data_Process()
{
    //数据处理过程
    memcpy(&Supercap_Data, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Struct_Supercap_CAN_Data));    

    Data.Chassis_Actual_Power = (float)Supercap_Data.Chassis_Actual_Power / 10.0f;
    Data.Supercap_Buffer_Power = (float)Supercap_Data.Supercap_Buffer_Power / 100.0f;
    Data.Supercap_Charge_Percentage = (float)Supercap_Data.Supercap_Charge_Percentage;
    Data.Supercup_Control_Level_Status = Supercap_Data.Supercup_Control_Level_Status;
    Data.Supercap_Current_Energy_Consumption = (float)Supercap_Data.Supercap_Current_Energy_Consumption / 100.0f;
}
/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Supercap::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Flag ++;
    Data_Process();
}
/**
 * @brief TIM定时器中断定期检测超级电容是否存活
 * 
 */
void Class_Supercap::TIM_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过超级电容数据
    if (Flag == Pre_Flag)
    {
        //超级电容断开连接
        Supercap_Status = Supercap_Status_DISABLE;
    }
    else
    {
        //超级电容保持连接
        Supercap_Status = Supercap_Status_ENABLE;
    }
    Pre_Flag = Flag;
}
/**
 * @brief 
 * 
 */
void Class_Supercap::Output()
{
    switch (PowerLimit_Type)
    {
    case PowerLimit_Type_Referee_BufferPower:
    {
        Chassis_Device_LimitPower = Referee_BufferPower_Output + Referee_MaxPower;
        Limit_Power = Referee_BufferPower_Output + Referee_MaxPower;
    }
    break;
    case PowerLimit_Type_Supercap_BufferPower:
    {
        Chassis_Device_LimitPower = Supercap_BufferPower_Output + Referee_MaxPower;
        Limit_Power = Supercap_LimitBufferPower_Output + Referee_MaxPower;
    }
    break;
    }
    //给超电can发送打包
    Set_Supercap_Control_Status((Enum_Supercap_Control_Status)SuperCap);
    Set_Limit_Power(Limit_Power);
    memcpy(CAN_Tx_Data, &Supercap_Tx_Data, sizeof(Struct_Supercap_Tx_Data));
    //给舵小板can发送打包
    memcpy(CAN1_0x01E_Tx_Data, &Chassis_Device_LimitPower, sizeof(float));
	memcpy(CAN1_0x01E_Tx_Data+4,&(Data.Chassis_Actual_Power),sizeof(float));
}

void Class_Supercap::Use_SuperCap_Strategy()
{
    // 超级电容策略 超电一直处于使能状态
    // 1.使用裁判系统缓冲环（斜率不易过大 功率控制讲究细水长流）（如果软件功率限制不住，莫慌，超电会帮你补一些）
    // 2.使用超级电容缓冲环并添加缓冲环充能策略（缓冲只加不减 ->充能）
    if (Get_Supercap_Status() == Supercap_Status_ENABLE)
    {
        switch (Supercap_Usage_Stratage)
        {
        case Supercap_Usage_Stratage_Referee_BufferPower:
        {
            Referee_BufferPower_Output = 0.5f * (Referee_BufferPower - 30.0f);
            Math_Constrain(&Referee_BufferPower_Output, -30.0f, 30.0f);
            Set_PowerLimit_Type(PowerLimit_Type_Referee_BufferPower);
        }
        break;
        case Supercap_Usage_Stratage_Supercap_BufferPower:
        {
            Supercap_BufferPower_Output = Data.Supercap_Buffer_Power;
            Supercap_LimitBufferPower_Output = 0.5f * (Referee_BufferPower - 30.0f);
            Math_Constrain(&Supercap_LimitBufferPower_Output,0.0f,30.0f);
            Set_PowerLimit_Type(PowerLimit_Type_Supercap_BufferPower);
        }
        break;
        }
    }
    else
    {
        Set_PowerLimit_Type(PowerLimit_Type_Referee_BufferPower);
    }

}

/**
 * @brief TIM定时器修改发送缓冲区
 * 
 */
void Class_Supercap::TIM_Supercap_PeriodElapsedCallback()
{
    Use_SuperCap_Strategy();
    Output();
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
