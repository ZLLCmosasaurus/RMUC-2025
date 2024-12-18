/**
 * @file drv_uart.c
 * @author yssickjgd (1345578933@qq.com)
 * @brief ·ՓCUT-Robotlab¸Đ´µĕARTͨЅ³õʼ»¯ӫŤփÁ÷³̍
 * @version 0.1
 * @date 2023-08-29 0.1 23ȼ¼¾¶¨¸半 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_usb.h"
#include "usbd_cdc_if.h"
//#include "tsk_config_and_callback.h"
/* Private macros ------------------------------------------------------------*/
Struct_USB_Manage_Object MiniPC_USB_Manage_Object = {0};
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief ³õʼ»¯USB
 *
 * @param Callback_Function ´¦À󆂵÷º¯ʽ
 */
void USB_Init(Struct_USB_Manage_Object* MiniPC_USB_Manage_Object, USB_Call_Back __Callback_Function)
{
	MiniPC_USB_Manage_Object->Callback_Function = __Callback_Function;
}

/**
 * @brief USBµĔIM¶¨ʱƷ֐¶Ϸ¢ˍ»ص÷º¯ʽ
 *
 */
void TIM_USB_PeriodElapsedCallback(Struct_USB_Manage_Object* MiniPC_USB_Manage_Object)
{
	CDC_Transmit_FS(MiniPC_USB_Manage_Object->Tx_Buffer, MiniPC_USB_Manage_Object->Tx_Buffer_Length+2);  //֡ͷ+֡β
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
