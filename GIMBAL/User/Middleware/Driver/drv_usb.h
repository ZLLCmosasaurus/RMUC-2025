/**
 * @file drv_uart.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的UART通信初始化与配置流程
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DRV_USB_H
#define DRV_USB_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

// 缓冲区字节长度
#define USB_BUFFER_SIZE 48

/* Exported types ------------------------------------------------------------*/


/**
 * @brief USB通信接收回调函数数据类型
 *
 */
typedef void (*USB_Call_Back)(uint8_t *Buffer, uint32_t Length);


/**
 * @brief USB通信处理结构体
 *
 */
struct Struct_USB_Manage_Object
{
    uint8_t Tx_Buffer[USB_BUFFER_SIZE];
    uint8_t Rx_Buffer[USB_BUFFER_SIZE];
    uint16_t Rx_Buffer_Length;
	  uint16_t Tx_Buffer_Length;
	  USB_Call_Back Callback_Function;
};

/* Exported variables --------------------------------------------------------*/


/* Exported function declarations --------------------------------------------*/

void USB_Init(Struct_USB_Manage_Object* MiniPC_USB_Manage_Object, USB_Call_Back __Callback_Function);
void TIM_USB_PeriodElapsedCallback(Struct_USB_Manage_Object* MiniPC_USB_Manage_Object);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
