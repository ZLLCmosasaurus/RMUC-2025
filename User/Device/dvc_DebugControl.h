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

enum Enum_DebugControl_Control_Status : uint8_t
{
    DebugControl_Control_Status_CONNECT = 0,
    DebugControl_Control_Status_RESET = 1,
    DebugControl_Control_Status_YAW = 2,
    DebugControl_Control_Status_TENSION = 3,
    DebugControl_Control_Status_RELOAD = 4,
    DebugControl_Control_Status_SHOOT = 5,
};


struct Struct_DebugControl_TxData
{
    uint8_t head;
    float now_yaw;
    float now_tension;
    uint8_t tear;
} __attribute__((packed));

struct Struct_DebugControl_RxData
{
    uint8_t head;
    Enum_DebugControl_Control_Status status;
    float target_yaw;
    float target_tension;
    uint8_t tear;
} __attribute__((packed));



/* Private variables ---------------------------------------------------------*/
class Class_DebugControl
{
    public:

        void Init(UART_HandleTypeDef *__UART_Handler);
        void DebugControl_Rx_Callback(uint8_t *Buffer, uint16_t Length);
        void DebugControl_Tx_Callback(float _now_yaw, float _now_tension);
        void DebugControl_Data_Process(uint8_t *Buffer, uint16_t Length);
        
        uint8_t Debug_Start_Flag = 0;

        inline void Set_Now_Yaw(float _now_yaw);
        inline void Set_Now_Tension(float _now_tension);
        inline float Get_Target_Yaw();
        inline float Get_Target_Tension();
        inline Enum_DebugControl_Control_Status Get_DebugControl_Status();

    private:
        Struct_UART_Manage_Object *UART_Manage_Object;

        Struct_DebugControl_RxData DebugControl_RxData;
        Struct_DebugControl_TxData DebugControl_TxData;

        //当前时刻接收flag
        uint32_t DebugControl_Flag = 0;
        //前一时刻接收flag
        uint32_t Pre_DebugControl_Flag = 0;

        
};
/* Private function declarations ---------------------------------------------*/


void Class_DebugControl::Set_Now_Yaw(float _now_yaw)
{
    DebugControl_TxData.now_yaw = _now_yaw;
}

void Class_DebugControl::Set_Now_Tension(float _now_tension)
{
    DebugControl_TxData.now_tension = _now_tension;
}

float Class_DebugControl::Get_Target_Yaw()
{
    return (DebugControl_RxData.target_yaw);
}

float Class_DebugControl::Get_Target_Tension()
{
    return (DebugControl_RxData.target_tension);
}

Enum_DebugControl_Control_Status Class_DebugControl::Get_DebugControl_Status()
{
    return (DebugControl_RxData.status);
}

/* Function prototypes -------------------------------------------------------*/

#endif

/************************ COPYRIGHT(C) ZLLC **************************/
