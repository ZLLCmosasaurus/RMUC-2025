/**
 * @file drv_can.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的CAN通信初始化与配置流程
 * @version 0.1
 * @date 2022-08-02
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DRV_CAN_H
#define DRV_CAN_H


/* Includes ------------------------------------------------------------------*/

#include "stdlib.h"
#include "string.h"
#include "stm32f4xx_hal.h"
/* Exported macros -----------------------------------------------------------*/
#ifdef __cplusplus
// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE    0
#define CAN_REMOTE_TYPE  1

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN接收的信息结构体
 *
 */
struct Struct_CAN_Rx_Buffer
{
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
};

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer *);

/**
 * @brief CAN通信处理结构体
 *
 */
struct Struct_CAN_Manage_Object
{
    CAN_HandleTypeDef *CAN_Handler;
    Struct_CAN_Rx_Buffer Rx_Buffer;
    CAN_Call_Back Callback_Function;
};

    struct CAN_Massage_Unit
    {
        CAN_HandleTypeDef *hcan;
        uint16_t ID;
        uint8_t *Data;
        uint16_t Length;
    };
/* Exported variables ---------------------------------------------------------*/

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern  Struct_CAN_Manage_Object CAN1_Manage_Object;
extern  Struct_CAN_Manage_Object CAN2_Manage_Object;

extern uint8_t CAN1_0x1ff_Tx_Data[];
extern uint8_t CAN1_0x200_Tx_Data[];
extern uint8_t CAN1_0x2ff_Tx_Data[];
extern uint8_t CAN1_0xxf1_Tx_Data[];
extern uint8_t CAN1_0xxf2_Tx_Data[];
extern uint8_t CAN1_0xxf3_Tx_Data[];
extern uint8_t CAN1_0xxf4_Tx_Data[];
extern uint8_t CAN1_0xxf5_Tx_Data[];
extern uint8_t CAN1_0xxf6_Tx_Data[];
extern uint8_t CAN1_0xxf7_Tx_Data[];
extern uint8_t CAN1_0xxf8_Tx_Data[];

extern uint8_t CAN2_0x1ff_Tx_Data[];
extern uint8_t CAN2_0x200_Tx_Data[];
extern uint8_t CAN2_0x2ff_Tx_Data[];
extern uint8_t CAN2_0xxf1_Tx_Data[];
extern uint8_t CAN2_0xxf2_Tx_Data[];
extern uint8_t CAN2_0xxf3_Tx_Data[];
extern uint8_t CAN2_0xxf4_Tx_Data[];
extern uint8_t CAN2_0xxf5_Tx_Data[];
extern uint8_t CAN2_0xxf6_Tx_Data[];
extern uint8_t CAN2_0xxf7_Tx_Data[];
extern uint8_t CAN2_0xxf8_Tx_Data[];

extern uint8_t CAN_Supercap_Tx_Data[];
extern uint8_t CAN2_Gimbal_Tx_Chassis_Data[];  //云台给底盘发送缓冲区
extern uint8_t CAN2_Chassis_Tx_Gimbal_Data[];   //底盘给云台发送缓冲区

extern uint8_t CAN1_0x1a_Tx_Streeing_Wheel_A_data[8];
extern uint8_t CAN1_0x1b_Tx_Streeing_Wheel_B_data[8];
extern uint8_t CAN1_0x1c_Tx_Streeing_Wheel_C_data[8];
extern uint8_t CAN1_0x1d_Tx_Streeing_Wheel_D_data[8];
extern uint8_t CAN1_0x01E_Tx_Data[8];
/*********LK电机 控制缓冲区***********/
extern uint8_t CAN1_0x141_Tx_Data[8];
extern uint8_t CAN1_0x142_Tx_Data[8];
extern uint8_t CAN1_0x143_Tx_Data[8];
extern uint8_t CAN1_0x144_Tx_Data[8];
extern uint8_t CAN1_0x145_Tx_Data[8];
extern uint8_t CAN1_0x146_Tx_Data[8];
extern uint8_t CAN1_0x147_Tx_Data[8];
extern uint8_t CAN1_0x148_Tx_Data[8];

extern uint8_t CAN2_0x141_Tx_Data[8];    
extern uint8_t CAN2_0x142_Tx_Data[8];
extern uint8_t CAN2_0x143_Tx_Data[8];
extern uint8_t CAN2_0x144_Tx_Data[8];
extern uint8_t CAN2_0x145_Tx_Data[8];    
extern uint8_t CAN2_0x146_Tx_Data[8];
extern uint8_t CAN2_0x147_Tx_Data[8];
extern uint8_t CAN2_0x148_Tx_Data[8];

/* Exported function declarations ---------------------------------------------*/

void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function);

uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);

void TIM_CAN_PeriodElapsedCallback();

#endif
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
