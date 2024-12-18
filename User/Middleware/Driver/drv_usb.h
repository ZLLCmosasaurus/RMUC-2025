/**
 * @file drv_uart.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief ·ՓCUT-Robotlab¸Đ´µĕARTͨЅ³õʼ»¯ӫŤփÁ÷³̍
 * @version 0.1
 * @date 2023-08-29 0.1 23ȼ¼¾¶¨¸半 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DRV_USB_H
#define DRV_USB_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif


/* Exported macros -----------------------------------------------------------*/

// »º³凸ז½ڳ¤¶ȍ
#define USB_BUFFER_SIZE 48

/* Exported types ------------------------------------------------------------*/


/**
 * @brief USBͨЅ½ӊջص÷º¯ʽʽ¾݀Ѝ
 *
 */
typedef void (*USB_Call_Back)(uint8_t *Buffer, uint32_t Length);


/**
 * @brief USBͨЅ´¦À󨒹¹̥
 *
 */
typedef struct 
{
    uint8_t Tx_Buffer[USB_BUFFER_SIZE];
    uint8_t Rx_Buffer[USB_BUFFER_SIZE];
    uint16_t Rx_Buffer_Length;
	  uint16_t Tx_Buffer_Length;
	  USB_Call_Back Callback_Function;
}Struct_USB_Manage_Object;

/* Exported variables --------------------------------------------------------*/

extern Struct_USB_Manage_Object MiniPC_USB_Manage_Object;
/* Exported function declarations --------------------------------------------*/

void USB_Init(Struct_USB_Manage_Object* MiniPC_USB_Manage_Object, USB_Call_Back __Callback_Function);
void TIM_USB_PeriodElapsedCallback(Struct_USB_Manage_Object* MiniPC_USB_Manage_Object);

#ifdef __cplusplus
}
#endif
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
