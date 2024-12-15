/**
 * @file drv_i2c.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 仿照SCUT-Robotlab改写的I2C通信初始化与配置流程
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DRV_I2C_H
#define DRV_I2C_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "main.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief IIC通信接收回调函数数据类型
 *
 */
typedef void (*IIC_Call_Back)(uint8_t* Tx_Buffer, uint8_t* Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length);

/* Exported variables --------------------------------------------------------*/

extern I2C_HandleTypeDef hi2c3;

enum Enum_IIC_Read_Write
{
    IIC_READ = 0,
    IIC_WRITE = 1
};


struct Struct_IIC_Manage_Object
{
    I2C_HandleTypeDef *IIC_Handler;
    uint16_t DevAddress;
    uint16_t MemAddress;
    uint16_t MemAddSize;
    uint8_t *pData;
    IIC_Call_Back Callback_Function;
};

/* Exported function declarations --------------------------------------------*/

extern Struct_IIC_Manage_Object IIC1_Manage_Object;
extern Struct_IIC_Manage_Object IIC2_Manage_Object;
extern Struct_IIC_Manage_Object IIC3_Manage_Object;

void IIC_Init(I2C_HandleTypeDef *hi2c, IIC_Call_Back Callback_Function);
uint8_t IIC_Send_Receive_Data(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout,Enum_IIC_Read_Write Read_Write);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
