/**
 * @file dvc_DebugControl.cpp
 * @author lez 
 * @brief 飞镖调试控制逻辑
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */
#ifndef TSK_DEBUGCONTROL_H
#define TSK_DEBUGCONTROL_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"
#include "drv_uart.h"
#include "stm32f4xx.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
#pragma pack(1)
union Struct_DebugControl_TxData
{
    uint8_t head;
    float now_yaw;
    float now_tension;
    uint8_t tear;
};

union Struct_DebugControl_RxData
{
    uint8_t head;
    uint8_t status;
    float target_yaw;
    float target_tension;
    uint8_t tear;
};
#pragma pack()


/* Private variables ---------------------------------------------------------*/
class Class_DebugControl
{
    public:

        void Init(UART_HandleTypeDef *__UART_Handler);
        void DebugControl_Rx_Callback(uint8_t *Buffer, uint16_t Length);
        void DebugControl_Tx_Callback(float _now_yaw, float _now_tension);

    private:
        Struct_UART_Manage_Object *UART_Manage_Object;

        Struct_DebugControl_RxData DebugControl_RxData;
        Struct_DebugControl_TxData DebugControl_TxData;

        //当前时刻接收flag
        uint32_t DebugControl_Flag = 0;
        //前一时刻接收flag
        uint32_t Pre_DebugControl_Flag = 0;

        void DebugControl_Data_Process(uint8_t *Buffer, uint16_t Length);
    
};
/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

#endif

/************************ COPYRIGHT(C) ZLLC **************************/
