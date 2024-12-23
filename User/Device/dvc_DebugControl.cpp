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

/* Includes ------------------------------------------------------------------*/

 #include "dvc_DebugControl.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
void Class_DebugControl::Init(UART_HandleTypeDef * __huart)
{
    //dr16串口
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
    else if (__huart->Instance == UART7)
    {
        UART_Manage_Object = &UART7_Manage_Object;
    }
    else
    {
        while (1)
        {
        }
    }
}


void Class_DebugControl::DebugControl_Rx_Callback(uint8_t *Buffer, uint16_t Length)
{
    this->DebugControl_Data_Process(Buffer, Length);
}

void Class_DebugControl::DebugControl_Tx_Callback(float _now_yaw, float _now_tension)
{
    DebugControl_TxData.head = 0xA5;
    DebugControl_TxData.now_yaw = _now_yaw;
    DebugControl_TxData.now_tension = _now_tension;
    DebugControl_TxData.tear = 0x5A;
    UART_Manage_Object->Tx_Buffer_Length = sizeof(Struct_DebugControl_TxData);
    memcpy(UART_Manage_Object->Tx_Buffer, &DebugControl_TxData, sizeof(Struct_DebugControl_TxData));
}


void Class_DebugControl::DebugControl_Data_Process(uint8_t *Buffer, uint16_t Length)
{
    if(Length != sizeof(Struct_DebugControl_RxData))
    {
        return;
    }
    Struct_DebugControl_RxData* _rx_data = (Struct_DebugControl_RxData*)UART_Manage_Object->Rx_Buffer;
    if (_rx_data->head == 0xA5 && _rx_data->tear == 0x5A)
    {
        DebugControl_Flag++;
        if(_rx_data->status == DebugControl_Control_Status_CONNECT)
        {
            Debug_Start_Flag = 1;
        }
        memcpy(&DebugControl_RxData, UART_Manage_Object->Rx_Buffer, sizeof(Struct_DebugControl_RxData));
    }        
}

/* Function prototypes -------------------------------------------------------*/



/************************ COPYRIGHT(C) ZLLC **************************/
