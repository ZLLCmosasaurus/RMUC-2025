/**
 * @file drv_uart.c
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的UART通信初始化与配置流程
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_usb.h"
#include "usbd_cdc_if.h"
#include "tsk_config_and_callback.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化USB
 *
 * @param Callback_Function 处理回调函数
 */
void USB_Init(Struct_USB_Manage_Object* MiniPC_USB_Manage_Object, USB_Call_Back __Callback_Function)
{
	MiniPC_USB_Manage_Object->Callback_Function = __Callback_Function;
}

/**
 * @brief USB的TIM定时器中断发送回调函数
 *
 */
void TIM_USB_PeriodElapsedCallback(Struct_USB_Manage_Object* MiniPC_USB_Manage_Object)
{
	CDC_Transmit_FS(MiniPC_USB_Manage_Object->Tx_Buffer, MiniPC_USB_Manage_Object->Tx_Buffer_Length+2);  //帧头+帧尾
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
